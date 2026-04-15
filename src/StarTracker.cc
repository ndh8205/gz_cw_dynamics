// Star tracker plugin for gz-sim.
//
// Produces body-in-ECI attitude quaternion (geometry_msgs/QuaternionStamped)
// with small-angle Gaussian noise.  Composition:
//
//   q_body_in_eci = q_lvlh_in_eci (from /chief/eci_state)  *  q_body_in_lvlh
//
// Noise is applied as a small-angle rotation:
//   delta_q ~ (1, theta_x/2, theta_y/2, theta_z/2),  theta_i ~ N(0, sigma)
//
// SDF:
//   <plugin filename="gz_star_tracker-system"
//           name="gz_cw_dynamics::StarTracker">
//     <link_name>deputy_link</link_name>
//     <chief_state_topic>/chief/eci_state</chief_state_topic>
//     <output_topic>/deputy/star_tracker/attitude</output_topic>
//     <frame_id>deputy_body</frame_id>
//     <angle_noise_deg>0.05</angle_noise_deg>
//     <update_rate>10</update_rate>
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
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

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

class StarTracker
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
{
public:
  ~StarTracker() override
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
      gzerr << "[StarTracker] must attach to a <model>." << std::endl;
      return;
    }

    const std::string linkName =
        sdf->Get<std::string>("link_name", std::string("base_link")).first;
    this->topic =
        sdf->Get<std::string>("output_topic",
                              std::string("/star_tracker/attitude")).first;
    this->chiefTopic =
        sdf->Get<std::string>("chief_state_topic",
                              std::string("/chief/eci_state")).first;
    this->frameId =
        sdf->Get<std::string>("frame_id", std::string("body")).first;
    const double sigmaDeg =
        sdf->Get<double>("angle_noise_deg", 0.05).first;
    this->sigmaRad = sigmaDeg * M_PI / 180.0;
    const double hz = sdf->Get<double>("update_rate", 10.0).first;
    this->minPeriod = (hz > 0.0) ? (1.0 / hz) : 0.1;
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
      gzerr << "[StarTracker] link [" << linkName << "] not found." << std::endl;
      return;
    }
    this->link = gz::sim::Link(linkEntity);

    if (!rclcpp::ok())
    {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
    std::string nodeName = "star_tracker" + this->topic;
    for (auto &c : nodeName) if (c == '/') c = '_';
    this->rosNode = std::make_shared<rclcpp::Node>(nodeName);
    this->rosPub =
        this->rosNode->create_publisher<geometry_msgs::msg::QuaternionStamped>(
            this->topic, rclcpp::SensorDataQoS());
    this->rosSub =
        this->rosNode->create_subscription<nav_msgs::msg::Odometry>(
            this->chiefTopic, rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
              std::lock_guard<std::mutex> lk(this->chiefMx);
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
    gzmsg << "[StarTracker] configured: link=" << linkName
          << ", topic=" << this->topic
          << ", chief=" << this->chiefTopic
          << ", sigma=" << sigmaDeg << " deg"
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
    if (!poseOpt.has_value()) return;
    const auto q_body_lvlh = poseOpt->Rot();   // body-to-world (LVLH)

    gz::math::Quaterniond q_lvlh_eci;
    bool have = false;
    {
      std::lock_guard<std::mutex> lk(this->chiefMx);
      q_lvlh_eci = this->qLvlhInEci;
      have = this->hasChief;
    }
    if (!have) return;

    // Composition: q_body_in_eci  (rotate body -> lvlh -> eci).
    const gz::math::Quaterniond q_body_eci = q_lvlh_eci * q_body_lvlh;

    // Small-angle noise.
    std::normal_distribution<double> nd(0.0, this->sigmaRad);
    const double tx = nd(this->rng);
    const double ty = nd(this->rng);
    const double tz = nd(this->rng);
    const gz::math::Quaterniond dq(1.0, tx/2, ty/2, tz/2);

    gz::math::Quaterniond q_meas = dq * q_body_eci;
    q_meas.Normalize();

    geometry_msgs::msg::QuaternionStamped msg;
    msg.header.frame_id = "eci";   // reference frame
    const auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(
                           info.simTime).count();
    msg.header.stamp.sec     = static_cast<int32_t>(nanos / 1000000000LL);
    msg.header.stamp.nanosec = static_cast<uint32_t>(nanos % 1000000000LL);
    msg.quaternion.x = q_meas.X();
    msg.quaternion.y = q_meas.Y();
    msg.quaternion.z = q_meas.Z();
    msg.quaternion.w = q_meas.W();
    this->rosPub->publish(msg);
  }

private:
  gz::sim::Model model;
  gz::sim::Link  link;

  std::string topic;
  std::string chiefTopic;
  std::string frameId;
  double sigmaRad{0.0};
  double minPeriod{0.1};
  double lastPublishSec{-1.0};
  bool configured{false};

  std::mutex chiefMx;
  gz::math::Quaterniond qLvlhInEci{1, 0, 0, 0};
  bool hasChief{false};

  std::mt19937 rng;

  rclcpp::Node::SharedPtr rosNode;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr rosPub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr rosSub;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> rosExecutor;
  std::thread rosThread;
};

}  // namespace gz_cw_dynamics

GZ_ADD_PLUGIN(gz_cw_dynamics::StarTracker,
              gz::sim::System,
              gz_cw_dynamics::StarTracker::ISystemConfigure,
              gz_cw_dynamics::StarTracker::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(gz_cw_dynamics::StarTracker,
                    "gz::sim::systems::StarTracker")
