// Clohessy-Wiltshire pseudo-force plugin for gz-sim (Harmonic / gz-sim8).
//
// Treats the Gazebo world frame as the LVLH (Hill) frame of a chief satellite.
// Applies F_cw = m * [ 2 n vy + 3 n^2 x, -2 n vx, -n^2 z ] to the deputy link
// every PreUpdate via Link::AddWorldForce. DART then integrates
// m a = F_cw + F_thrust, reproducing the CW equations of relative motion.
//
// SDF:
//   <plugin filename="gz_cw_dynamics-system"
//           name="gz_cw_dynamics::CWDynamics">
//     <link_name>nasa_satellite_link</link_name>
//     <mean_motion>1.0959e-3</mean_motion>
//     <initial_velocity>0.02740 0.0 0.04746</initial_velocity>
//     <pseudo_accel_topic>/deputy/cw_pseudo_accel</pseudo_accel_topic>
//   </plugin>

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/vector3d.pb.h>
#include <gz/common/Console.hh>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

namespace gz_cw_dynamics
{

class CWDynamics
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager & /*eventMgr*/) override
  {
    this->model = gz::sim::Model(entity);
    if (!this->model.Valid(ecm))
    {
      gzerr << "[CWDynamics] Plugin must be attached to a <model>."
            << std::endl;
      return;
    }

    const std::string linkName =
        sdf->Get<std::string>("link_name", std::string("base_link")).first;
    this->meanMotion =
        sdf->Get<double>("mean_motion", 1.0959e-3).first;

    if (sdf->HasElement("initial_velocity"))
    {
      this->initialVelocity =
          sdf->Get<gz::math::Vector3d>("initial_velocity");
      this->hasInitialVelocity = true;
    }

    // --- J2 augmentation (Schweighart-Sedwick 2002) --------------------
    this->enableJ2 = sdf->Get<bool>("enable_j2", false).first;
    if (this->enableJ2)
    {
      const double J2     = sdf->Get<double>("j2",           1.082627e-3).first;
      const double Re     = sdf->Get<double>("earth_radius", 6378.1363  ).first;
      const double a      = sdf->Get<double>("sma",          6923.137   ).first;
      const double i_deg  = sdf->Get<double>("inclination_deg",
                                             97.5736787997506).first;
      const double i_rad  = i_deg * M_PI / 180.0;

      const double Re_over_a = Re / a;
      const double s = 3.0 * J2 / 8.0 *
                       Re_over_a * Re_over_a *
                       (1.0 + 3.0 * std::cos(2.0 * i_rad));
      this->cSS  = std::sqrt(1.0 + s);
      this->cSS2 = this->cSS * this->cSS;

      gzmsg << "[CWDynamics] J2 on: J2=" << J2
            << ", Re/a=" << Re_over_a
            << ", i=" << i_deg << " deg"
            << ", s=" << s
            << ", c=" << this->cSS << std::endl;
    }

    const auto linkEntity = this->model.LinkByName(ecm, linkName);
    if (linkEntity == gz::sim::kNullEntity)
    {
      gzerr << "[CWDynamics] Link [" << linkName
            << "] not found in model [" << this->model.Name(ecm) << "]."
            << std::endl;
      return;
    }
    this->link = gz::sim::Link(linkEntity);
    this->link.EnableVelocityChecks(ecm, true);

    const auto inertialComp =
        ecm.Component<gz::sim::components::Inertial>(linkEntity);
    if (!inertialComp)
    {
      gzerr << "[CWDynamics] Inertial component missing on link ["
            << linkName << "]." << std::endl;
      return;
    }
    this->mass = inertialComp->Data().MassMatrix().Mass();

    const std::string topic =
        sdf->Get<std::string>("pseudo_accel_topic",
                              std::string("/deputy/cw_pseudo_accel")).first;
    this->pub = this->node.Advertise<gz::msgs::Vector3d>(topic);

    this->configured = true;
    gzmsg << "[CWDynamics] configured: n=" << this->meanMotion
          << " rad/s, link=" << linkName
          << ", mass=" << this->mass << " kg"
          << (this->hasInitialVelocity ? ", v0 set" : "")
          << ", topic=" << topic << std::endl;
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override
  {
    if (!this->configured || info.paused) return;

    const auto poseOpt = this->link.WorldPose(ecm);
    const auto velOpt  = this->link.WorldLinearVelocity(ecm);
    if (!poseOpt.has_value() || !velOpt.has_value()) return;

    const auto &r = poseOpt->Pos();
    const auto &v = velOpt.value();

    const double n  = this->meanMotion;
    const double n2 = n * n;

    // Clohessy-Wiltshire pseudo-acceleration, Hill frame:
    //   x: radial, y: along-track, z: cross-track.
    // With enable_j2, use Schweighart-Sedwick linearization.
    gz::math::Vector3d aCW;
    if (this->enableJ2)
    {
      const double nc = n * this->cSS;
      const double kx = (5.0 * this->cSS2 - 2.0) * n2;  // radial stiffness
      const double qz2 = n2 * this->cSS2;               // cross-track freq^2
      aCW.Set(
         2.0 * nc * v.Y() + kx * r.X(),
        -2.0 * nc * v.X(),
        -qz2 * r.Z());
    }
    else
    {
      aCW.Set(
         2.0 * n * v.Y() + 3.0 * n2 * r.X(),
        -2.0 * n * v.X(),
        -n2 * r.Z());
    }

    gz::math::Vector3d force = this->mass * aCW;

    // Inject initial velocity as an impulsive force over one timestep.
    // F_impulse * dt = m * v0  ->  F_impulse = m * v0 / dt
    if (this->hasInitialVelocity && !this->initialVelocityApplied)
    {
      const double dt =
          std::chrono::duration<double>(info.dt).count();
      if (dt > 0.0)
      {
        force += (this->mass / dt) * this->initialVelocity;
        this->initialVelocityApplied = true;
        gzmsg << "[CWDynamics] initial velocity impulse injected: "
              << this->initialVelocity << " m/s over dt=" << dt << "s"
              << std::endl;
      }
    }

    this->link.AddWorldForce(ecm, force);

    // Throttle pseudo-accel publishing to ~100 Hz regardless of physics rate.
    const auto simTime = std::chrono::duration<double>(info.simTime).count();
    if (simTime - this->lastPublishSec >= 0.01)
    {
      gz::msgs::Vector3d msg;
      msg.set_x(aCW.X());
      msg.set_y(aCW.Y());
      msg.set_z(aCW.Z());
      this->pub.Publish(msg);
      this->lastPublishSec = simTime;
    }
  }

private:
  gz::sim::Model model;
  gz::sim::Link link;

  double meanMotion{1.0959e-3};
  double mass{1.0};

  gz::math::Vector3d initialVelocity{0.0, 0.0, 0.0};
  bool hasInitialVelocity{false};
  bool initialVelocityApplied{false};
  bool configured{false};

  bool   enableJ2{false};
  double cSS{1.0};
  double cSS2{1.0};

  gz::transport::Node node;
  gz::transport::Node::Publisher pub;
  double lastPublishSec{-1.0};
};

}  // namespace gz_cw_dynamics

GZ_ADD_PLUGIN(gz_cw_dynamics::CWDynamics,
              gz::sim::System,
              gz_cw_dynamics::CWDynamics::ISystemConfigure,
              gz_cw_dynamics::CWDynamics::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz_cw_dynamics::CWDynamics,
                    "gz::sim::systems::CWDynamics")
