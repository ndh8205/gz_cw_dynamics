// GPS receiver plugin (ECI-frame output) for gz-sim.
//
// Gazebo world is the chief's LVLH frame. Deputy's ECI state is obtained by
// composing the deputy's LVLH state with the chief's ECI state received on
// /chief/eci_state (nav_msgs/Odometry):
//
//   r_deputy_eci = r_chief_eci + R_lvlh_to_eci * r_deputy_lvlh
//   v_deputy_eci = v_chief_eci
//                  + omega_lvlh/eci_eci  x  (R_lvlh_to_eci * r_deputy_lvlh)
//                  + R_lvlh_to_eci * v_deputy_lvlh
//
// Publishes nav_msgs/Odometry on <output_topic> with header.frame_id = "eci",
// child_frame_id = "deputy". Zero-mean Gaussian noise is added independently
// to position (sigma_pos_m) and velocity (sigma_vel_mps).
//
// SDF:
//   <plugin filename="gz_gps-system" name="gz_cw_dynamics::Gps">
//     <link_name>deputy_link</link_name>
//     <chief_state_topic>/chief/eci_state</chief_state_topic>
//     <output_topic>/deputy/gps/odometry</output_topic>
//     <sigma_pos_m>5.0</sigma_pos_m>
//     <sigma_vel_mps>0.05</sigma_vel_mps>
//     <update_rate>1</update_rate>
//     <seed>0</seed>
//   </plugin>

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/common/Console.hh>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>

namespace gz_cw_dynamics
{

class Gps
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
{
public:
  ~Gps() override
  {
    if (this->rosExecutor) this->rosExecutor->cancel();
    if (this->rosThread.joinable()) this->rosThread.join();
  }

  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager & /*evt*/) override
  {
    this->model = gz::sim::Model(entity);
    if (!this->model.Valid(ecm))
    {
      gzerr << "[Gps] must attach to a <model>." << std::endl;
      return;
    }

    const std::string linkName =
        sdf->Get<std::string>("link_name", std::string("base_link")).first;
    this->topic =
        sdf->Get<std::string>("output_topic",
                              std::string("/gps/odometry")).first;
    this->chiefTopic =
        sdf->Get<std::string>("chief_state_topic",
                              std::string("/chief/eci_state")).first;
    this->sigmaPos =
        sdf->Get<double>("sigma_pos_m", 5.0).first;
    this->sigmaVel =
        sdf->Get<double>("sigma_vel_mps", 0.05).first;
    const double hz = sdf->Get<double>("update_rate", 1.0).first;
    this->minPeriod = (hz > 0.0) ? (1.0 / hz) : 1.0;
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
      gzerr << "[Gps] link [" << linkName << "] not found." << std::endl;
      return;
    }
    this->link = gz::sim::Link(linkEntity);
    this->link.EnableVelocityChecks(ecm, true);

    if (!rclcpp::ok())
    {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
    std::string nodeName = "gps" + this->topic;
    for (auto &c : nodeName) if (c == '/') c = '_';
    this->rosNode = std::make_shared<rclcpp::Node>(nodeName);
    this->rosPub = this->rosNode->create_publisher<nav_msgs::msg::Odometry>(
        this->topic, rclcpp::SensorDataQoS());
    this->rosSub =
        this->rosNode->create_subscription<nav_msgs::msg::Odometry>(
            this->chiefTopic, rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
              std::lock_guard<std::mutex> lk(this->chiefMx);
              this->rChief.Set(msg->pose.pose.position.x,
                               msg->pose.pose.position.y,
                               msg->pose.pose.position.z);
              this->vChief.Set(msg->twist.twist.linear.x,
                               msg->twist.twist.linear.y,
                               msg->twist.twist.linear.z);
              this->omegaLvlhEci.Set(msg->twist.twist.angular.x,
                                     msg->twist.twist.angular.y,
                                     msg->twist.twist.angular.z);
              this->qLvlhInEci = gz::math::Quaterniond(
                  msg->pose.pose.orientation.w,
                  msg->pose.pose.orientation.x,
                  msg->pose.pose.orientation.y,
                  msg->pose.pose.orientation.z);
              this->hasChief = true;
            });
    this->rosExecutor =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    this->rosExecutor->add_node(this->rosNode);
    this->rosThread = std::thread([this](){ this->rosExecutor->spin(); });

    this->configured = true;
    gzmsg << "[Gps] configured: link=" << linkName
          << ", topic=" << this->topic
          << ", chief=" << this->chiefTopic
          << ", sigma_pos=" << this->sigmaPos << " m"
          << ", sigma_vel=" << this->sigmaVel << " m/s"
          << ", rate=" << hz << " Hz" << std::endl;
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override
  {
    if (!this->configured || info.paused) return;
    const double t = std::chrono::duration<double>(info.simTime).count();
    if (t - this->lastPublishSec < this->minPeriod) return;
    this->lastPublishSec = t;

    const auto poseOpt = this->link.WorldPose(ecm);
    const auto velOpt  = this->link.WorldLinearVelocity(ecm);
    if (!poseOpt.has_value() || !velOpt.has_value()) return;

    gz::math::Vector3d rChiefLocal, vChiefLocal, omegaLocal;
    gz::math::Quaterniond qLocal;
    bool have = false;
    {
      std::lock_guard<std::mutex> lk(this->chiefMx);
      rChiefLocal  = this->rChief;
      vChiefLocal  = this->vChief;
      omegaLocal   = this->omegaLvlhEci;
      qLocal       = this->qLvlhInEci;
      have         = this->hasChief;
    }
    if (!have) return;

    // Deputy LVLH state from Gazebo.
    const gz::math::Vector3d r_lvlh = poseOpt->Pos();
    const gz::math::Vector3d v_lvlh = velOpt.value();

    // Rotate LVLH to ECI.
    const gz::math::Vector3d r_lvlh_in_eci = qLocal.RotateVector(r_lvlh);
    const gz::math::Vector3d v_lvlh_in_eci = qLocal.RotateVector(v_lvlh);

    // ECI kinematics: deputy_eci = chief_eci + (rot of lvlh->eci) * local.
    // v_eci uses transport theorem: omega x r added on top.
    const gz::math::Vector3d r_deputy_eci = rChiefLocal + r_lvlh_in_eci;
    const gz::math::Vector3d v_deputy_eci =
        vChiefLocal + omegaLocal.Cross(r_lvlh_in_eci) + v_lvlh_in_eci;

    // Noise.
    std::normal_distribution<double> np(0.0, this->sigmaPos);
    std::normal_distribution<double> nv(0.0, this->sigmaVel);
    const gz::math::Vector3d r_meas(
        r_deputy_eci.X() + np(this->rng),
        r_deputy_eci.Y() + np(this->rng),
        r_deputy_eci.Z() + np(this->rng));
    const gz::math::Vector3d v_meas(
        v_deputy_eci.X() + nv(this->rng),
        v_deputy_eci.Y() + nv(this->rng),
        v_deputy_eci.Z() + nv(this->rng));

    nav_msgs::msg::Odometry msg;
    msg.header.frame_id = "eci";
    msg.child_frame_id  = "deputy";
    const auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(
                           info.simTime).count();
    msg.header.stamp.sec     = static_cast<int32_t>(nanos / 1000000000LL);
    msg.header.stamp.nanosec = static_cast<uint32_t>(nanos % 1000000000LL);
    msg.pose.pose.position.x = r_meas.X();
    msg.pose.pose.position.y = r_meas.Y();
    msg.pose.pose.position.z = r_meas.Z();
    msg.pose.pose.orientation.w = 1.0;     // GPS does not report attitude
    msg.twist.twist.linear.x = v_meas.X();
    msg.twist.twist.linear.y = v_meas.Y();
    msg.twist.twist.linear.z = v_meas.Z();
    this->rosPub->publish(msg);
  }

private:
  gz::sim::Model model;
  gz::sim::Link  link;

  std::string topic;
  std::string chiefTopic;
  double sigmaPos{5.0};
  double sigmaVel{0.05};
  double minPeriod{1.0};
  double lastPublishSec{-1.0};
  bool configured{false};

  std::mutex chiefMx;
  gz::math::Vector3d rChief{0, 0, 0};
  gz::math::Vector3d vChief{0, 0, 0};
  gz::math::Vector3d omegaLvlhEci{0, 0, 0};
  gz::math::Quaterniond qLvlhInEci{1, 0, 0, 0};
  bool hasChief{false};

  std::mt19937 rng;

  rclcpp::Node::SharedPtr rosNode;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rosPub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr rosSub;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> rosExecutor;
  std::thread rosThread;
};

}  // namespace gz_cw_dynamics

GZ_ADD_PLUGIN(gz_cw_dynamics::Gps,
              gz::sim::System,
              gz_cw_dynamics::Gps::ISystemConfigure,
              gz_cw_dynamics::Gps::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(gz_cw_dynamics::Gps,
                    "gz::sim::systems::Gps")
