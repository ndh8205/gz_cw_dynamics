// Reaction wheel plugin for gz-sim.
//
// Listens on a ROS 2 std_msgs/Float32 topic for commanded wheel torque
// [N·m] and applies it to the specified revolute joint via
// components::JointForceCmd. By Newton's third law, DART produces the
// opposite torque on the parent body - that is what actually controls
// spacecraft attitude.
//
// SDF:
//   <plugin filename="gz_reaction_wheel_ros2-system"
//           name="gz_cw_dynamics::ReactionWheel">
//     <joint_name>wheel_x_joint</joint_name>
//     <topic>/deputy/rw/x/cmd</topic>
//     <max_torque>0.01</max_torque>   <!-- N·m, clamped -->
//   </plugin>

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/Joint.hh>
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

    const std::string jointName =
        sdf->Get<std::string>("joint_name", std::string("wheel_joint")).first;
    this->topic =
        sdf->Get<std::string>("topic", std::string("/rw/cmd")).first;
    this->maxTorque =
        sdf->Get<double>("max_torque", 0.01).first;

    this->jointEntity = this->model.JointByName(ecm, jointName);
    if (this->jointEntity == gz::sim::kNullEntity)
    {
      gzerr << "[ReactionWheel] joint [" << jointName << "] not found."
            << std::endl;
      return;
    }

    if (!rclcpp::ok())
    {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
    std::string nodeName = "rw" + this->topic;
    for (auto &c : nodeName) if (c == '/') c = '_';
    this->rosNode = std::make_shared<rclcpp::Node>(nodeName);
    this->rosSub = this->rosNode->create_subscription<std_msgs::msg::Float32>(
        this->topic, rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
          double t = static_cast<double>(msg->data);
          if (t >  this->maxTorque) t =  this->maxTorque;
          if (t < -this->maxTorque) t = -this->maxTorque;
          this->torque.store(t);
        });
    this->rosExecutor =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    this->rosExecutor->add_node(this->rosNode);
    this->rosThread = std::thread([this](){ this->rosExecutor->spin(); });

    this->configured = true;
    gzmsg << "[ReactionWheel] configured: joint=" << jointName
          << ", topic=" << this->topic
          << ", max_torque=" << this->maxTorque << " Nm" << std::endl;
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override
  {
    if (!this->configured || info.paused) return;
    const double t = this->torque.load();

    auto *cmdComp =
        ecm.Component<gz::sim::components::JointForceCmd>(this->jointEntity);
    if (cmdComp)
    {
      cmdComp->Data() = std::vector<double>{t};
    }
    else
    {
      ecm.CreateComponent(this->jointEntity,
          gz::sim::components::JointForceCmd({t}));
    }
  }

private:
  gz::sim::Model  model;
  gz::sim::Entity jointEntity{gz::sim::kNullEntity};
  std::string topic;
  double maxTorque{0.01};
  bool configured{false};

  std::atomic<double> torque{0.0};

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
