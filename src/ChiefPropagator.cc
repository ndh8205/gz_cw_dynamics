// Chief orbit propagator (Layer A) for gz-sim.
//
// Runs at the world level. Given the chief satellite's initial orbital
// elements + epoch, analytically propagates its ECI state (assumes ECC = 0
// - no Kepler solver needed) and publishes:
//   <chief_state_topic>     nav_msgs/Odometry   (r_eci [m], v_eci [m/s])
//   <sun_lvlh_topic>        geometry_msgs/Vector3Stamped  (unit vector, LVLH)
//
// Does NOT apply any force. Purely informational — the deputy dynamics
// remain untouched.
//
// SDF:
//   <plugin filename="gz_chief_propagator-system"
//           name="gz_cw_dynamics::ChiefPropagator">
//     <sma>6923.137</sma>
//     <ecc>0.0</ecc>
//     <inc_deg>97.5736787997506</inc_deg>
//     <raan_deg>226.932075641711</raan_deg>
//     <aop_deg>171.130489033293</aop_deg>
//     <ta_deg>0.0</ta_deg>
//     <mu>3.986004418e14</mu>    <!-- m^3/s^2 (SI) -->
//     <sun_eci>0.89 0.42 0.18</sun_eci>
//     <chief_state_topic>/chief/eci_state</chief_state_topic>
//     <sun_lvlh_topic>/chief/sun_vector_lvlh</sun_lvlh_topic>
//     <update_rate>10</update_rate>
//   </plugin>

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/LightCmd.hh>
#include <gz/sim/Conversions.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Matrix3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Pose3.hh>
#include <gz/common/Console.hh>

#include <sdf/Light.hh>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

namespace gz_cw_dynamics
{

class ChiefPropagator
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  ~ChiefPropagator() override
  {
    if (this->rosExecutor) this->rosExecutor->cancel();
    if (this->rosThread.joinable()) this->rosThread.join();
  }

  void Configure(const gz::sim::Entity & /*entity*/,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager & /*ecm*/,
                 gz::sim::EventManager & /*evt*/) override
  {
    this->a_m  = sdf->Get<double>("sma", 6923.137).first * 1000.0;  // km -> m
    this->ecc  = sdf->Get<double>("ecc", 0.0).first;
    this->inc  = sdf->Get<double>("inc_deg",  97.5736787997506).first
                  * M_PI / 180.0;
    this->raan = sdf->Get<double>("raan_deg", 226.932075641711).first
                  * M_PI / 180.0;
    this->aop  = sdf->Get<double>("aop_deg",  171.130489033293).first
                  * M_PI / 180.0;
    this->ta0  = sdf->Get<double>("ta_deg",   0.0).first * M_PI / 180.0;
    this->mu   = sdf->Get<double>("mu",       3.986004418e14).first;

    const auto sunEci = sdf->Get<gz::math::Vector3d>(
        "sun_eci", gz::math::Vector3d(1.0, 0.0, 0.0)).first;
    if (sunEci.Length() < 1e-9)
    {
      gzerr << "[ChiefPropagator] sun_eci has zero length." << std::endl;
      return;
    }
    this->sunEci = sunEci;
    this->sunEci.Normalize();

    this->stateTopic =
        sdf->Get<std::string>("chief_state_topic",
                              std::string("/chief/eci_state")).first;
    this->sunTopic =
        sdf->Get<std::string>("sun_lvlh_topic",
                              std::string("/chief/sun_vector_lvlh")).first;

    const double hz = sdf->Get<double>("update_rate", 10.0).first;
    this->minPeriod = (hz > 0.0) ? (1.0 / hz) : 0.1;

    this->sunVisualName =
        sdf->Get<std::string>("sun_visual_name",
                              std::string("sun_visual")).first;
    this->sunLightName =
        sdf->Get<std::string>("sun_light_name",
                              std::string("sun")).first;
    this->sunVisualDistance =
        sdf->Get<double>("sun_visual_distance", 500.0).first;

    if (this->ecc > 1e-6)
    {
      gzwarn << "[ChiefPropagator] ECC=" << this->ecc
             << " > 0, but propagation only handles ECC=0 analytically."
             << std::endl;
    }

    this->n = std::sqrt(this->mu / (this->a_m * this->a_m * this->a_m));
    this->v0 = std::sqrt(this->mu / this->a_m);

    // ROS 2 publishers.
    if (!rclcpp::ok())
    {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
    this->rosNode = std::make_shared<rclcpp::Node>("chief_propagator");
    this->statePub = this->rosNode->create_publisher<nav_msgs::msg::Odometry>(
        this->stateTopic, rclcpp::SensorDataQoS());
    this->sunPub =
        this->rosNode->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            this->sunTopic, rclcpp::SensorDataQoS());
    this->rosExecutor =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    this->rosExecutor->add_node(this->rosNode);
    this->rosThread = std::thread([this](){ this->rosExecutor->spin(); });

    this->configured = true;
    gzmsg << "[ChiefPropagator] configured: a=" << (this->a_m/1000)
          << " km, n=" << this->n
          << " rad/s, T=" << (2.0*M_PI/this->n) << " s"
          << ", sun_eci=" << this->sunEci
          << ", state_topic=" << this->stateTopic
          << ", sun_topic=" << this->sunTopic << std::endl;
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override
  {
    if (!this->configured || info.paused) return;

    const double t = std::chrono::duration<double>(info.simTime).count();
    if (t - this->lastPublishSec < this->minPeriod) return;
    this->lastPublishSec = t;

    // Circular orbit: true anomaly == mean anomaly.
    const double nu = this->ta0 + this->n * t;
    const double u  = this->aop + nu;   // argument of latitude

    const double cu = std::cos(u);
    const double su = std::sin(u);
    const double cR = std::cos(this->raan);
    const double sR = std::sin(this->raan);
    const double ci = std::cos(this->inc);
    const double si = std::sin(this->inc);

    // r_eci = a * ... (standard ECC=0 closed form)
    const gz::math::Vector3d r_eci(
        this->a_m * ( cu*cR - su*ci*sR),
        this->a_m * ( cu*sR + su*ci*cR),
        this->a_m * ( su*si));

    const gz::math::Vector3d v_eci(
        this->v0 * (-su*cR - cu*ci*sR),
        this->v0 * (-su*sR + cu*ci*cR),
        this->v0 * ( cu*si));

    // LVLH axes expressed in ECI.
    const gz::math::Vector3d xhat = r_eci.Normalized();
    const gz::math::Vector3d zhat = r_eci.Cross(v_eci).Normalized();
    const gz::math::Vector3d yhat = zhat.Cross(xhat);

    // R_eci_to_lvlh: rows are the LVLH axes in ECI.
    const gz::math::Matrix3d R_eci2lvlh(
        xhat.X(), xhat.Y(), xhat.Z(),
        yhat.X(), yhat.Y(), yhat.Z(),
        zhat.X(), zhat.Y(), zhat.Z());

    const gz::math::Vector3d sun_lvlh = R_eci2lvlh * this->sunEci;

    // Scene entity manipulation (sun_visual pose, sun light direction)
    // disabled: both broke the GUI rendering in this gz-sim build, even
    // with SetChanged/LightCmd. The plugin now only publishes telemetry
    // (ECI state + sun vector in LVLH). Sun visualization can be driven
    // from a separate node via gz service /world/<w>/set_pose.
    (void)sun_lvlh;

    // NOTE: dynamic directional-light rewriting was found to break scene
    // rendering in this gz-sim build (both Light component edits and
    // LightCmd pushes caused the GUI to go black). The sun_visual sphere
    // already gives students a clear visual cue of sun direction, so we
    // leave the static <light name="sun"> alone. If dynamic lighting is
    // needed later, drive it from a separate node via the
    // /world/<name>/light_config service instead.

    // --- Publish Odometry (chief ECI state).
    nav_msgs::msg::Odometry odo;
    const auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(
                           info.simTime).count();
    odo.header.stamp.sec     = static_cast<int32_t>(nanos / 1000000000LL);
    odo.header.stamp.nanosec = static_cast<uint32_t>(nanos % 1000000000LL);
    odo.header.frame_id      = "eci";
    odo.child_frame_id       = "chief";
    odo.pose.pose.position.x = r_eci.X();
    odo.pose.pose.position.y = r_eci.Y();
    odo.pose.pose.position.z = r_eci.Z();
    // orientation = rotation from ECI to LVLH as quaternion.
    // Columns of R_lvlh_to_eci = LVLH basis vectors in ECI.
    const gz::math::Matrix3d R_lvlh2eci(
        xhat.X(), yhat.X(), zhat.X(),
        xhat.Y(), yhat.Y(), zhat.Y(),
        xhat.Z(), yhat.Z(), zhat.Z());
    const gz::math::Quaterniond q_lvlh_in_eci(R_lvlh2eci);
    odo.pose.pose.orientation.x = q_lvlh_in_eci.X();
    odo.pose.pose.orientation.y = q_lvlh_in_eci.Y();
    odo.pose.pose.orientation.z = q_lvlh_in_eci.Z();
    odo.pose.pose.orientation.w = q_lvlh_in_eci.W();
    odo.twist.twist.linear.x = v_eci.X();
    odo.twist.twist.linear.y = v_eci.Y();
    odo.twist.twist.linear.z = v_eci.Z();
    // angular = omega_lvlh/eci expressed in ECI = n * zhat
    odo.twist.twist.angular.x = this->n * zhat.X();
    odo.twist.twist.angular.y = this->n * zhat.Y();
    odo.twist.twist.angular.z = this->n * zhat.Z();
    this->statePub->publish(odo);

    // --- Publish sun vector (LVLH).
    geometry_msgs::msg::Vector3Stamped sun_msg;
    sun_msg.header.stamp = odo.header.stamp;
    sun_msg.header.frame_id = "lvlh";
    sun_msg.vector.x = sun_lvlh.X();
    sun_msg.vector.y = sun_lvlh.Y();
    sun_msg.vector.z = sun_lvlh.Z();
    this->sunPub->publish(sun_msg);
  }

private:
  // Orbital elements (radians + meters).
  double a_m{6923137.0};
  double ecc{0.0};
  double inc{0.0};
  double raan{0.0};
  double aop{0.0};
  double ta0{0.0};
  double mu{3.986004418e14};
  double n{0.0};
  double v0{0.0};

  gz::math::Vector3d sunEci{1.0, 0.0, 0.0};

  std::string stateTopic;
  std::string sunTopic;

  std::string sunVisualName{"sun_visual"};
  std::string sunLightName{"sun"};
  double      sunVisualDistance{500.0};
  gz::sim::Entity sunVisualEntity{gz::sim::kNullEntity};
  gz::sim::Entity sunLightEntity{gz::sim::kNullEntity};

  double minPeriod{0.1};
  double lastPublishSec{-1.0};
  bool   configured{false};

  rclcpp::Node::SharedPtr rosNode;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr statePub;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr sunPub;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> rosExecutor;
  std::thread rosThread;
};

}  // namespace gz_cw_dynamics

GZ_ADD_PLUGIN(gz_cw_dynamics::ChiefPropagator,
              gz::sim::System,
              gz_cw_dynamics::ChiefPropagator::ISystemConfigure,
              gz_cw_dynamics::ChiefPropagator::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz_cw_dynamics::ChiefPropagator,
                    "gz::sim::systems::ChiefPropagator")
