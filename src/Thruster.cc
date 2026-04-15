// Reference thruster plugin for gz-sim (Harmonic).
//
// Subscribes to a ROS 2 std_msgs/Float32 topic carrying throttle in [0, 1].
// Every PreUpdate applies  F = throttle * max_force * dir_body  at
// position_body relative to the link origin, converted to world frame via
// the link's current attitude.
//
// Attach one <plugin> block per thruster to the same <model>. For the
// seminar a typical deputy gets 12 thrusters (+/- on each of 6 faces).
//
// SDF:
//   <plugin filename="gz_thruster_ros2-system"
//           name="gz_cw_dynamics::Thruster">
//     <link_name>deputy_link</link_name>
//     <topic>/deputy/thruster/fx_plus/cmd</topic>
//     <position>0.5 0 0</position>   <!-- body frame [m] -->
//     <direction>1 0 0</direction>    <!-- body frame unit vector -->
//     <max_force>1.0</max_force>      <!-- Newton -->
//   </plugin>

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <gz/common/Console.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <atomic>
#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace gz_cw_dynamics
{

class Thruster
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  ~Thruster() override
  {
    if (this->rosExecutor)
    {
      this->rosExecutor->cancel();
    }
    if (this->rosThread.joinable())
    {
      this->rosThread.join();
    }
  }

  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager & /*eventMgr*/) override
  {
    this->model = gz::sim::Model(entity);
    if (!this->model.Valid(ecm))
    {
      gzerr << "[Thruster] must be attached to a <model>." << std::endl;
      return;
    }

    const std::string linkName =
        sdf->Get<std::string>("link_name", std::string("base_link")).first;
    this->topic =
        sdf->Get<std::string>("topic",
                              std::string("/thruster/cmd")).first;
    this->positionBody =
        sdf->Get<gz::math::Vector3d>("position",
                                     gz::math::Vector3d::Zero).first;
    this->directionBody =
        sdf->Get<gz::math::Vector3d>("direction",
                                     gz::math::Vector3d(1, 0, 0)).first;
    if (this->directionBody.Length() < 1e-9)
    {
      gzerr << "[Thruster] <direction> has zero length." << std::endl;
      return;
    }
    this->directionBody.Normalize();
    this->maxForce = sdf->Get<double>("max_force", 1.0).first;

    const auto linkEntity = this->model.LinkByName(ecm, linkName);
    if (linkEntity == gz::sim::kNullEntity)
    {
      gzerr << "[Thruster] Link [" << linkName << "] not found." << std::endl;
      return;
    }
    this->link = gz::sim::Link(linkEntity);

    // --- Start ROS 2 subscription ---------------------------------------
    if (!rclcpp::ok())
    {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }

    // Node name derived from topic to keep multiple thrusters unique.
    std::string nodeName = "thruster" + this->topic;
    std::replace(nodeName.begin(), nodeName.end(), '/', '_');
    this->rosNode = std::make_shared<rclcpp::Node>(nodeName);

    this->rosSub = this->rosNode->create_subscription<std_msgs::msg::Float32>(
        this->topic, rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
          double t = static_cast<double>(msg->data);
          if (t < 0.0) t = 0.0;
          if (t > 1.0) t = 1.0;
          this->throttle.store(t);
        });

    this->rosExecutor =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    this->rosExecutor->add_node(this->rosNode);
    this->rosThread = std::thread([this]() { this->rosExecutor->spin(); });

    this->configured = true;
    gzmsg << "[Thruster] configured: link=" << linkName
          << ", topic=" << this->topic
          << ", pos=" << this->positionBody
          << ", dir=" << this->directionBody
          << ", Fmax=" << this->maxForce << " N" << std::endl;
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override
  {
    if (!this->configured || info.paused) return;
    const double t = this->throttle.load();
    if (t <= 0.0) return;

    const auto pose = this->link.WorldPose(ecm);
    if (!pose.has_value()) return;

    // Body-frame thrust force -> world-frame (attitude rotation only).
    const gz::math::Vector3d forceBody =
        (t * this->maxForce) * this->directionBody;
    const gz::math::Vector3d forceWorld = pose->Rot().RotateVector(forceBody);

    // gz-sim AddWorldForce(force_world, offset_body) applies the force at
    // the given offset expressed in the link frame.
    this->link.AddWorldForce(ecm, forceWorld, this->positionBody);
  }

private:
  gz::sim::Model model;
  gz::sim::Link  link;

  std::string        topic;
  gz::math::Vector3d positionBody{gz::math::Vector3d::Zero};
  gz::math::Vector3d directionBody{1, 0, 0};
  double             maxForce{1.0};
  bool               configured{false};

  std::atomic<double> throttle{0.0};

  rclcpp::Node::SharedPtr rosNode;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rosSub;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> rosExecutor;
  std::thread rosThread;
};

}  // namespace gz_cw_dynamics

GZ_ADD_PLUGIN(gz_cw_dynamics::Thruster,
              gz::sim::System,
              gz_cw_dynamics::Thruster::ISystemConfigure,
              gz_cw_dynamics::Thruster::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz_cw_dynamics::Thruster,
                    "gz::sim::systems::Thruster")
