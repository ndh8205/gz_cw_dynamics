// Reaction wheel plugin (virtual torque mode) for gz-sim.
//
// Applies commanded torque directly to the body link (no physical wheel
// link/joint required). This allows attaching via <include> without
// modifying the satellite model.  Functionally equivalent to a physical
// RW for attitude control — students see the same topic interface and
// the body rotates in response.
//
// SDF:
//   <plugin filename="gz_reaction_wheel_ros2-system"
//           name="gz_cw_dynamics::ReactionWheel">
//     <link_name>base_link</link_name>
//     <axis>0 0 1</axis>           <!-- body-frame torque axis -->
//     <topic>/deputy/rw/z/cmd</topic>
//     <max_torque>0.01</max_torque>
//   </plugin>

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/math/Vector3.hh>
#include <gz/common/Console.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <atomic>
#include <algorithm>
#include <memory>
#include <string>
#include <thread>

namespace gz_cw_dynamics
{

class ReactionWheel
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  ~ReactionWheel() override
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
      gzerr << "[ReactionWheel] must attach to a <model>." << std::endl;
      return;
    }

    const std::string linkName =
        sdf->Get<std::string>("link_name", std::string("base_link")).first;
    this->topic =
        sdf->Get<std::string>("topic", std::string("/rw/cmd")).first;
    this->maxTorque =
        sdf->Get<double>("max_torque", 0.01).first;
    this->axis =
        sdf->Get<gz::math::Vector3d>("body_axis",
                                     gz::math::Vector3d(0, 0, 1)).first;
    if (this->axis.Length() > 1e-9)
      this->axis.Normalize();

    const auto linkEntity = this->model.LinkByName(ecm, linkName);
    if (linkEntity == gz::sim::kNullEntity)
    {
      gzerr << "[ReactionWheel] link [" << linkName << "] not found."
            << std::endl;
      return;
    }
    this->link = gz::sim::Link(linkEntity);

    if (!rclcpp::ok())
    {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
    std::string nodeName = "rw" + this->topic;
    for (auto &c : nodeName) if (c == '/') c = '_';
    this->rosNode = std::make_shared<rclcpp::Node>(nodeName);
    this->rosSub = this->rosNode->create_subscription<std_msgs::msg::Float32>(
        this->topic, 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
          double t = static_cast<double>(msg->data);
          if (t >  this->maxTorque) t =  this->maxTorque;
          if (t < -this->maxTorque) t = -this->maxTorque;
          this->torqueCmd.store(t);
        });
    this->rosExecutor =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    this->rosExecutor->add_node(this->rosNode);
    this->rosThread = std::thread([this](){ this->rosExecutor->spin(); });

    this->configured = true;
    gzmsg << "[ReactionWheel] configured: link=" << linkName
          << ", axis=" << this->axis
          << ", topic=" << this->topic
          << ", max_torque=" << this->maxTorque << " Nm" << std::endl;
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override
  {
    if (!this->configured || info.paused) return;

    const double cmd = this->torqueCmd.load();
    if (std::abs(cmd) < 1e-12) return;

    // Torque in body frame along the configured axis.
    // AddWorldWrench expects world-frame torque, so rotate from body.
    const auto poseOpt = this->link.WorldPose(ecm);
    if (!poseOpt.has_value()) return;

    const gz::math::Vector3d torqueBody = cmd * this->axis;
    const gz::math::Vector3d torqueWorld =
        poseOpt->Rot().RotateVector(torqueBody);

    this->link.AddWorldWrench(ecm,
        gz::math::Vector3d::Zero,   // no force
        torqueWorld);                // torque only
  }

private:
  gz::sim::Model model;
  gz::sim::Link  link;
  std::string topic;
  gz::math::Vector3d axis{0, 0, 1};
  double maxTorque{0.01};
  bool configured{false};

  std::atomic<double> torqueCmd{0.0};

  rclcpp::Node::SharedPtr rosNode;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rosSub;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> rosExecutor;
  std::thread rosThread;
};

}  // namespace gz_cw_dynamics

GZ_ADD_PLUGIN(gz_cw_dynamics::ReactionWheel,
              gz::sim::System,
              gz_cw_dynamics::ReactionWheel::ISystemConfigure,
              gz_cw_dynamics::ReactionWheel::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz_cw_dynamics::ReactionWheel,
                    "gz::sim::systems::ReactionWheel")
