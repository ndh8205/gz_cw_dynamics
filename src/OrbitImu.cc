// Orbit-aware IMU plugin for gz-sim (Harmonic).
//
// The Gazebo world frame in this workspace IS the chief's LVLH (Hill) frame,
// which rotates relative to inertial at rate n about +z. A real rate-gyro on
// a deputy whose body frame is aligned with LVLH therefore reads (0, 0, n),
// not zero as a naive Gazebo IMU would.
//
// Similarly, a real accelerometer in orbit only senses non-gravitational
// specific force. DART's reported WorldLinearAcceleration includes the CW
// pseudo-force we inject to simulate gravity gradient + Coriolis; we must
// subtract that before the output can be interpreted like a real IMU.
//
// Outputs sensor_msgs/Imu on <topic>:
//   angular_velocity       = omega_body/inertial expressed in body frame
//   linear_acceleration    = non-gravitational specific force in body frame
//   orientation            = body-to-world quaternion (LVLH-relative)
//
// SDF:
//   <plugin filename="gz_orbit_imu-system" name="gz_cw_dynamics::OrbitImu">
//     <link_name>deputy_link</link_name>
//     <mean_motion>1.0959e-3</mean_motion>
//     <topic>/deputy/imu/data</topic>
//     <frame_id>deputy_body</frame_id>
//     <pseudo_accel_topic>/deputy/cw_pseudo_accel</pseudo_accel_topic>
//     <update_rate>100</update_rate>
//   </plugin>

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/vector3d.pb.h>
#include <gz/common/Console.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>

namespace gz_cw_dynamics
{

class OrbitImu
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
{
public:
  ~OrbitImu() override
  {
    if (this->rosExecutor) this->rosExecutor->cancel();
    if (this->rosThread.joinable()) this->rosThread.join();
  }

  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager & /*eventMgr*/) override
  {
    this->model = gz::sim::Model(entity);
    if (!this->model.Valid(ecm))
    {
      gzerr << "[OrbitImu] must attach to a <model>." << std::endl;
      return;
    }

    const std::string linkName =
        sdf->Get<std::string>("link_name", std::string("base_link")).first;
    this->meanMotion =
        sdf->Get<double>("mean_motion", 0.0).first;
    this->topic =
        sdf->Get<std::string>("topic",
                              std::string("/imu/data")).first;
    this->frameId =
        sdf->Get<std::string>("frame_id", std::string("body")).first;
    const std::string pseudoTopic =
        sdf->Get<std::string>("pseudo_accel_topic",
                              std::string("/deputy/cw_pseudo_accel")).first;
    const double updateHz =
        sdf->Get<double>("update_rate", 100.0).first;
    this->minPeriod = (updateHz > 0.0) ? (1.0 / updateHz) : 0.0;

    // --- Noise / bias parameters (defaults = truth-only) --------------
    this->accelNoiseStd = sdf->Get<double>("accel_noise_stddev", 0.0).first;
    this->gyroNoiseStd  = sdf->Get<double>("gyro_noise_stddev",  0.0).first;
    this->accelBiasRW   = sdf->Get<double>("accel_bias_rw",      0.0).first;
    this->gyroBiasRW    = sdf->Get<double>("gyro_bias_rw",       0.0).first;
    this->accelBias     = sdf->Get<gz::math::Vector3d>(
        "accel_bias_init", gz::math::Vector3d::Zero).first;
    this->gyroBias      = sdf->Get<gz::math::Vector3d>(
        "gyro_bias_init",  gz::math::Vector3d::Zero).first;

    const int seed = sdf->Get<int>("seed", 0).first;
    if (seed == 0)
    {
      std::random_device rd;
      this->rng.seed(rd());
    }
    else
    {
      this->rng.seed(static_cast<std::mt19937::result_type>(seed));
    }

    const auto linkEntity = this->model.LinkByName(ecm, linkName);
    if (linkEntity == gz::sim::kNullEntity)
    {
      gzerr << "[OrbitImu] link [" << linkName << "] not found." << std::endl;
      return;
    }
    this->link = gz::sim::Link(linkEntity);
    this->link.EnableVelocityChecks(ecm, true);
    this->link.EnableAccelerationChecks(ecm, true);

    // gz-transport subscription to CW pseudo-accel.
    this->gzNode.Subscribe<gz::msgs::Vector3d>(
        pseudoTopic,
        [this](const gz::msgs::Vector3d &msg) {
          std::lock_guard<std::mutex> lk(this->accelMx);
          this->cwPseudoAccel.Set(msg.x(), msg.y(), msg.z());
          this->hasCwAccel = true;
        });

    // ROS 2 publisher.
    if (!rclcpp::ok())
    {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
    std::string nodeName = "orbit_imu" + this->topic;
    for (auto &c : nodeName) if (c == '/') c = '_';
    this->rosNode = std::make_shared<rclcpp::Node>(nodeName);
    this->rosPub = this->rosNode->create_publisher<sensor_msgs::msg::Imu>(
        this->topic, rclcpp::SensorDataQoS());
    this->rosExecutor =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    this->rosExecutor->add_node(this->rosNode);
    this->rosThread = std::thread([this](){ this->rosExecutor->spin(); });

    this->configured = true;
    gzmsg << "[OrbitImu] configured: link=" << linkName
          << ", n=" << this->meanMotion << " rad/s"
          << ", topic=" << this->topic
          << ", frame=" << this->frameId
          << ", pseudo_topic=" << pseudoTopic
          << ", rate=" << updateHz << " Hz" << std::endl;
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override
  {
    if (!this->configured || info.paused) return;

    const double simSec =
        std::chrono::duration<double>(info.simTime).count();
    if (simSec - this->lastPublishSec < this->minPeriod) return;
    const double dt = (this->lastPublishSec < 0.0)
                      ? this->minPeriod
                      : (simSec - this->lastPublishSec);
    this->lastPublishSec = simSec;

    const auto poseOpt = this->link.WorldPose(ecm);
    const auto wOpt    = this->link.WorldAngularVelocity(ecm);
    const auto aOpt    = this->link.WorldLinearAcceleration(ecm);
    if (!poseOpt.has_value() || !wOpt.has_value() || !aOpt.has_value())
      return;

    const auto &q = poseOpt->Rot();           // body -> world rotation

    // --- Gyro: omega_body/inertial expressed in body frame. -----------
    // omega_body/inertial_world = omega_body/world_world + omega_LVLH/I_world
    // omega_LVLH/I_world = (0, 0, n) by our Hill-frame convention.
    const gz::math::Vector3d w_bw_world = wOpt.value();
    const gz::math::Vector3d w_li_world(0.0, 0.0, this->meanMotion);
    const gz::math::Vector3d w_bi_body =
        q.RotateVectorReverse(w_bw_world + w_li_world);

    // --- Specific force: a_world - a_cw, then rotate to body. ----------
    gz::math::Vector3d a_cw(0, 0, 0);
    {
      std::lock_guard<std::mutex> lk(this->accelMx);
      if (this->hasCwAccel) a_cw = this->cwPseudoAccel;
    }
    const gz::math::Vector3d a_world = aOpt.value();
    const gz::math::Vector3d f_body_true =
        q.RotateVectorReverse(a_world - a_cw);

    // --- Bias random walk  b(k+1) = b(k) + sigma_rw * sqrt(dt) * N(0,1)
    const double sqrtDt = std::sqrt(dt > 0.0 ? dt : this->minPeriod);
    if (this->accelBiasRW > 0.0)
    {
      this->accelBias += gz::math::Vector3d(
          this->accelBiasRW * sqrtDt * this->rw(),
          this->accelBiasRW * sqrtDt * this->rw(),
          this->accelBiasRW * sqrtDt * this->rw());
    }
    if (this->gyroBiasRW > 0.0)
    {
      this->gyroBias += gz::math::Vector3d(
          this->gyroBiasRW * sqrtDt * this->rw(),
          this->gyroBiasRW * sqrtDt * this->rw(),
          this->gyroBiasRW * sqrtDt * this->rw());
    }

    // --- White noise  eta = sigma_n / sqrt(dt) * N(0,1)
    const double invSqrtDt = (sqrtDt > 0.0) ? (1.0 / sqrtDt) : 0.0;
    const gz::math::Vector3d eta_a = (this->accelNoiseStd * invSqrtDt) *
        gz::math::Vector3d(this->rw(), this->rw(), this->rw());
    const gz::math::Vector3d eta_g = (this->gyroNoiseStd * invSqrtDt) *
        gz::math::Vector3d(this->rw(), this->rw(), this->rw());

    // --- Measurement = truth + bias + noise ---------------------------
    const gz::math::Vector3d f_body_meas = f_body_true + this->accelBias + eta_a;
    const gz::math::Vector3d w_bi_meas   = w_bi_body   + this->gyroBias  + eta_g;

    // --- Publish ROS 2 Imu. -------------------------------------------
    sensor_msgs::msg::Imu msg;
    msg.header.frame_id = this->frameId;
    const auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(
                           info.simTime).count();
    msg.header.stamp.sec     = static_cast<int32_t>(nanos / 1000000000LL);
    msg.header.stamp.nanosec = static_cast<uint32_t>(nanos % 1000000000LL);

    msg.orientation.x = q.X();
    msg.orientation.y = q.Y();
    msg.orientation.z = q.Z();
    msg.orientation.w = q.W();

    msg.angular_velocity.x = w_bi_meas.X();
    msg.angular_velocity.y = w_bi_meas.Y();
    msg.angular_velocity.z = w_bi_meas.Z();

    msg.linear_acceleration.x = f_body_meas.X();
    msg.linear_acceleration.y = f_body_meas.Y();
    msg.linear_acceleration.z = f_body_meas.Z();

    // Covariances unknown -> leave zero (ROS convention).
    this->rosPub->publish(msg);
  }

private:
  gz::sim::Model model;
  gz::sim::Link  link;

  double meanMotion{0.0};
  std::string topic;
  std::string frameId;
  double minPeriod{0.01};
  double lastPublishSec{-1.0};
  bool   configured{false};

  std::mutex accelMx;
  gz::math::Vector3d cwPseudoAccel{0, 0, 0};
  bool hasCwAccel{false};

  // Noise / bias model (RW bias + white Gaussian noise).
  double accelNoiseStd{0.0};
  double gyroNoiseStd{0.0};
  double accelBiasRW{0.0};
  double gyroBiasRW{0.0};
  gz::math::Vector3d accelBias{0, 0, 0};
  gz::math::Vector3d gyroBias{0, 0, 0};
  std::mt19937 rng;
  std::normal_distribution<double> normDist{0.0, 1.0};
  double rw() { return this->normDist(this->rng); }

  gz::transport::Node gzNode;

  rclcpp::Node::SharedPtr rosNode;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr rosPub;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> rosExecutor;
  std::thread rosThread;
};

}  // namespace gz_cw_dynamics

GZ_ADD_PLUGIN(gz_cw_dynamics::OrbitImu,
              gz::sim::System,
              gz_cw_dynamics::OrbitImu::ISystemConfigure,
              gz_cw_dynamics::OrbitImu::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(gz_cw_dynamics::OrbitImu,
                    "gz::sim::systems::OrbitImu")
