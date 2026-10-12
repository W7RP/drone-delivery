// Phase 8 chase camera (docs/hero-clip.md 5): a world system that poses a static, massless
// camera model behind and above a target model every sim step, looking at it. The camera model
// has no inertia and no collision and is never in the physics of the target, so it cannot change
// the flight. Its pose follows the target's with a first-order lag in sim time, so the picture
// is the same whatever the wall speed.
//
// SDF (inside a world <plugin>):
//   <target>dd_vtol</target>        model to follow
//   <camera>dd_cam_chase</camera>   static model to pose
//   <back_m>14</back_m> <up_m>5</up_m> <side_m>3</side_m>  offset in the target's track frame
//   <lag_s>1.5</lag_s>              time constant of the position lag
//   <min_track_speed_mps>2</min_track_speed_mps>  below it the track is the target's heading
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Pose.hh>

namespace dd
{

class ChaseCamera final : public gz::sim::System, public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    target_ = sdf->Get<std::string>("target", target_).first;
    camera_ = sdf->Get<std::string>("camera", camera_).first;
    back_m_ = sdf->Get<double>("back_m", back_m_).first;
    up_m_ = sdf->Get<double>("up_m", up_m_).first;
    side_m_ = sdf->Get<double>("side_m", side_m_).first;
    lag_s_ = sdf->Get<double>("lag_s", lag_s_).first;
    min_track_speed_ = sdf->Get<double>("min_track_speed_mps", min_track_speed_).first;
  }

  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused) {return;}
    // The models are spawned after the world starts; look them up until both exist.
    if (!target_entity_) {target_entity_ = model(ecm, target_);}
    if (!camera_entity_) {camera_entity_ = model(ecm, camera_);}
    if (!target_entity_ || !camera_entity_) {return;}
    const auto target = target_entity_;
    const auto camera = camera_entity_;
    const auto * pose = ecm.Component<gz::sim::components::Pose>(*target);
    if (pose == nullptr) {return;}
    const gz::math::Vector3d p = pose->Data().Pos();
    const double t = std::chrono::duration<double>(info.simTime).count();
    // Track direction: the horizontal velocity, differenced from the pose, once the target
    // moves; its heading before that.
    double yaw = pose->Data().Rot().Yaw();
    if (last_target_ && t > last_t_) {
      const gz::math::Vector3d v = (p - *last_target_) / (t - last_t_);
      const double speed = std::hypot(v.X(), v.Y());
      if (speed > min_track_speed_) {track_yaw_ = std::atan2(v.Y(), v.X());}
    }
    if (!track_yaw_) {track_yaw_ = yaw;}
    yaw = *track_yaw_;
    const gz::math::Vector3d fwd{std::cos(yaw), std::sin(yaw), 0.0};
    const gz::math::Vector3d left{-std::sin(yaw), std::cos(yaw), 0.0};
    const gz::math::Vector3d want = p - fwd * back_m_ + left * side_m_ + gz::math::Vector3d{0, 0, up_m_};
    if (!cam_pos_) {
      cam_pos_ = want;
    } else if (t > last_t_) {
      const double a = 1.0 - std::exp(-(t - last_t_) / lag_s_);
      *cam_pos_ += (want - *cam_pos_) * a;
    }
    last_target_ = p;
    last_t_ = t;
    const gz::math::Vector3d look = p - *cam_pos_;
    const double horizontal = std::hypot(look.X(), look.Y());
    // Gazebo cameras look along +X; a positive pitch tilts that axis down.
    const gz::math::Quaterniond rot(0.0, std::atan2(-look.Z(), horizontal), std::atan2(look.Y(), look.X()));
    const gz::math::Pose3d cam_pose(*cam_pos_, rot);
    *ecm.Component<gz::sim::components::Pose>(*camera) = gz::sim::components::Pose(cam_pose);
    ecm.SetChanged(*camera, gz::sim::components::Pose::typeId,
      gz::sim::ComponentState::PeriodicChange);
  }

private:
  static std::optional<gz::sim::Entity> model(const gz::sim::EntityComponentManager & ecm,
    const std::string & name)
  {
    const gz::sim::Entity e = ecm.EntityByComponents(gz::sim::components::Model(),
        gz::sim::components::Name(name));
    if (e == gz::sim::kNullEntity) {return std::nullopt;}
    return e;
  }

  std::string target_ = "dd_vtol", camera_ = "dd_cam_chase";
  double back_m_ = 14.0, up_m_ = 5.0, side_m_ = 3.0, lag_s_ = 1.5, min_track_speed_ = 2.0;
  std::optional<gz::sim::Entity> target_entity_, camera_entity_;
  std::optional<gz::math::Vector3d> last_target_, cam_pos_;
  std::optional<double> track_yaw_;
  double last_t_ = 0.0;
};

}  // namespace dd

GZ_ADD_PLUGIN(dd::ChaseCamera, gz::sim::System, gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)
