// src/RectangleMover.cc

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/SystemLoader.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>

#include <vector>
#include <cmath>

namespace ignition::gazebo::systems
{
  class RectangleMover 
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate
  {
    /// \brief A single waypoint: time + desired pose.
    struct Waypoint
    {
      double time;
      ignition::math::Pose3d pose;
    };

    /// \brief The list of waypoints parsed from the SDF.
    std::vector<Waypoint> waypoints;

    /// \brief The Gazebo model this plugin is attached to.
    Model model{kNullEntity};

  public:
    /// \brief Read all <waypoint> elements
    void Configure(const Entity &entity,
                   const std::shared_ptr<const sdf::Element> &sdf,
                   EntityComponentManager & /*ecm*/,
                   EventManager & /*eventMgr*/) override
    {
      // Store the model
      this->model = Model(entity);

      // Iterate every child element
      for (auto elem = sdf->GetFirstElement(); elem; elem = elem->GetNextElement())
      {
        if (elem->GetName() == "waypoint")
        {
          double t = elem->Get<double>("time");
          ignition::math::Pose3d p = elem->Get<ignition::math::Pose3d>("pose");
          this->waypoints.push_back({t, p});
        }
      }
    }

    /// \brief Called each simulation iteration; interpolate & set pose.
    void PreUpdate(const UpdateInfo &info,
                   EntityComponentManager &ecm) override
    {
      if (this->waypoints.empty())
        return;

      // Current simulation time in seconds
      double simTime = std::chrono::duration<double>(info.simTime).count();

      // Loop around
      double totalDuration = this->waypoints.back().time;
      double tLoop = std::fmod(simTime, totalDuration);

      // Default to last waypoint
      ignition::math::Pose3d interpPose = this->waypoints.back().pose;

      // Find segment
      for (size_t i = 1; i < this->waypoints.size(); ++i)
      {
        if (tLoop < this->waypoints[i].time)
        {
          const auto &w0 = this->waypoints[i-1];
          const auto &w1 = this->waypoints[i];
          double t0 = w0.time, t1 = w1.time;
          double ratio = (tLoop - t0) / (t1 - t0);

          // Interpolate position
          auto p0 = w0.pose.Pos();
          auto p1 = w1.pose.Pos();
          ignition::math::Vector3d pos = p0 + (p1 - p0) * ratio;

          // Slerp orientation
          auto q0 = w0.pose.Rot();
          auto q1 = w1.pose.Rot();
          // static Quaterniond Slerp(T _t, const Quaterniond &_q1,
          //                          const Quaterniond &_q2, bool _shortestPath)
          ignition::math::Quaterniond rot =
            ignition::math::Quaterniond::Slerp(ratio, q0, q1, true);

          interpPose = ignition::math::Pose3d(pos, rot);
          break;
        }
      }

      // Command the model to the new pose
      // nueva: insertamos directamente el componente WorldPose
      ecm.SetComponentData<ignition::gazebo::components::WorldPose>(
          this->model.Entity(), interpPose);
    }
  };
}

// Register plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::systems::RectangleMover,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ignition::gazebo::systems::RectangleMover,
                          "rectangle_movement")