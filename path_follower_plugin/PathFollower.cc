
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/plugin/Register.hh>

#include <fstream>
#include <string>
#include <vector>

namespace path_follower
{
class PathFollower : public ignition::gazebo::System,
                     public ignition::gazebo::ISystemConfigure,
                     public ignition::gazebo::ISystemPreUpdate
{
public:
    PathFollower() = default;
    ~PathFollower() override = default;

    void Configure(const ignition::gazebo::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   ignition::gazebo::EntityComponentManager &_ecm,
                   ignition::gazebo::EventManager &/*_eventMgr*/) override
    {
        this->model = ignition::gazebo::Model(_entity);
        if (!this->model.Valid(_ecm)) {
            ignerr << "Plugin should be attached to a model entity. Failed to initialize." << std::endl;
            return;
        }

        auto modelName = this->model.Name(_ecm);

        if (!_sdf->HasElement("path_file")) {
            ignerr << "PathFollower plugin missing <path_file> element for model [" << modelName << "]" << std::endl;
            return;
        }
        this->path_file = _sdf->Get<std::string>("path_file");

        std::ifstream file(this->path_file);
        if (!file.is_open()) {
            ignerr << "Failed to open path file: " << this->path_file << " for model [" << modelName << "]" << std::endl;
            return;
        }

        double x, y;
        while (file >> x >> y) {
            this->waypoints.push_back(ignition::math::Vector2d(x, y));
        }

        if (this->waypoints.empty()) {
            ignerr << "No waypoints found in path file: " << this->path_file << " for model [" << modelName << "]" << std::endl;
            return;
        }

        ignmsg << "PathFollower loaded for [" << modelName << "]. " << this->waypoints.size() << " waypoints loaded." << std::endl;
    }

    void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                   ignition::gazebo::EntityComponentManager &_ecm) override
    {
        if (this->waypoints.empty() || _info.paused) {
            return;
        }

        auto poseComp = _ecm.Component<ignition::gazebo::components::Pose>(this->model.Entity());
        auto linVelComp = _ecm.Component<ignition::gazebo::components::LinearVelocity>(this->model.Entity());
        auto angVelComp = _ecm.Component<ignition::gazebo::components::AngularVelocity>(this->model.Entity());

        if (!poseComp) {
             // Wait for pose component to be created
            _ecm.CreateComponent(this->model.Entity(), ignition::gazebo::components::Pose());
            return;
        }

        // Create velocity components if they don't exist
        if (!linVelComp) {
            _ecm.CreateComponent(this->model.Entity(), ignition::gazebo::components::LinearVelocity());
        }
        if (!angVelComp) {
            _ecm.CreateComponent(this->model.Entity(), ignition::gazebo::components::AngularVelocity());
        }

        const ignition::math::Pose3d &current_pose = poseComp->Data();
        ignition::math::Vector2d current_pos(current_pose.Pos().X(), current_pose.Pos().Y());
        ignition::math::Vector2d target_waypoint = this->waypoints[this->target_waypoint_index];

        double distance_to_target = current_pos.Distance(target_waypoint);

        if (distance_to_target < this->waypoint_threshold) {
            this->target_waypoint_index++;
            if (this->target_waypoint_index >= this->waypoints.size()) {
                this->target_waypoint_index = 0; // Loop
            }
            return;
        }

        ignition::math::Vector2d dir = target_waypoint - current_pos;
        double target_angle = atan2(dir.Y(), dir.X());
        double angle_diff = ignition::math::Angle(target_angle - current_pose.Rot().Yaw()).Normalized().Radian();

        double angular_vel = this->kp_angular * angle_diff;
        double linear_vel = this->kp_linear * distance_to_target;

        if (linear_vel > this->max_speed) linear_vel = this->max_speed;
        if (std::abs(angle_diff) > (IGN_PI / 2.5)) linear_vel = 0;

        ignition::math::Vector3d body_lin_vel(linear_vel, 0, 0);
        ignition::math::Vector3d body_ang_vel(0, 0, angular_vel);

        // Update the component data
        if(linVelComp) *linVelComp = ignition::gazebo::components::LinearVelocity(body_lin_vel);
        if(angVelComp) *angVelComp = ignition::gazebo::components::AngularVelocity(body_ang_vel);
    }

private:
    ignition::gazebo::Model model{ignition::gazebo::kNullEntity};
    std::string path_file;
    std::vector<ignition::math::Vector2d> waypoints;
    unsigned int target_waypoint_index = 0;

    // --- Controller Parameters ---
    double waypoint_threshold = 0.4;
    double kp_linear = 0.7;
    double kp_angular = 2.0;
    double max_speed = 1.2;
};

} // namespace path_follower

// Register this plugin
IGNITION_ADD_PLUGIN(
    path_follower::PathFollower,
    ignition::gazebo::System,
    path_follower::PathFollower::ISystemConfigure,
    path_follower::PathFollower::ISystemPreUpdate
)
