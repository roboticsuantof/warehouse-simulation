// ActorAvoidance.cc
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Events.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Types.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/msgs/laserscan.pb.h>
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/transport/Node.hh>

using namespace ignition;
using namespace gazebo;

namespace
{
  double YawOf(const math::Quaterniond &q)
  {
    return q.Euler().Z();
  }

  double WrapPi(double a)
  {
    while (a > M_PI) a -= 2.0*M_PI;
    while (a < -M_PI) a += 2.0*M_PI;
    return a;
  }

  bool StartsWith(const std::string &s, const std::string &pfx)
  {
    return s.size() >= pfx.size() && s.compare(0, pfx.size(), pfx) == 0;
  }
}

// Datos por actor
struct TrackedActor
{
  Entity actor{kNullEntity};
  Entity proxy{kNullEntity};

  std::string actorName;
  std::string proxyName;
  std::string scanTopic;

  std::atomic<double> minRange{std::numeric_limits<double>::infinity()};
  std::atomic<double> leftMin{std::numeric_limits<double>::infinity()};
  std::atomic<double> rightMin{std::numeric_limits<double>::infinity()};

  ignition::math::Pose3d lastPose{ignition::math::Pose3d::Zero};
  bool hasLastPose{false};
};

class ActorAvoidance :
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate
{
public:
  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &) override
  {
    this->worldEntity = _entity;

    if (auto name = _ecm.Component<components::Name>(_entity))
      this->worldName = name->Data(); // ej. "industrial-warehouse"

    // Parámetros opcionales
    if (_sdf && _sdf->HasElement("actor_prefix"))
      this->actorPrefix = _sdf->Get<std::string>("actor_prefix");
    if (_sdf && _sdf->HasElement("proxy_prefix"))
      this->proxyPrefix = _sdf->Get<std::string>("proxy_prefix");
    if (_sdf && _sdf->HasElement("min_distance"))
      this->minDistance = _sdf->Get<double>("min_distance");
    if (_sdf && _sdf->HasElement("turn_rate"))
      this->turnRate = _sdf->Get<double>("turn_rate");
    if (_sdf && _sdf->HasElement("lateral_speed"))
      this->lateralSpeed = _sdf->Get<double>("lateral_speed");
    if (_sdf && _sdf->HasElement("sensor_height"))
      this->sensorHeight = _sdf->Get<double>("sensor_height");

    // Descubre actores/proxies
    DiscoverActorsAndProxies(_ecm);

    // Suscripciones
    for (auto &kv : this->actors)
    {
      auto taPtr = kv.second;
      if (!taPtr || taPtr->scanTopic.empty()) continue;

      std::function<void(const ignition::msgs::LaserScan&)> cb =
        [taPtr](const ignition::msgs::LaserScan &msg)
        {
          double m = std::numeric_limits<double>::infinity();
          double left = std::numeric_limits<double>::infinity();
          double right = std::numeric_limits<double>::infinity();

          const int n = msg.ranges_size();
          for (int i = 0; i < n; ++i)
          {
            const double r = static_cast<double>(msg.ranges(i));
            if (r <= 0.0) continue;
            if (r < m) m = r;
            if (i < n/2) left = std::min(left, r);
            else         right = std::min(right, r);
          }
          taPtr->minRange.store(m, std::memory_order_relaxed);
          taPtr->leftMin.store(left, std::memory_order_relaxed);
          taPtr->rightMin.store(right, std::memory_order_relaxed);
        };

      this->node.Subscribe(taPtr->scanTopic, cb);
    }
  }

  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;

    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (dt <= 0.0)
      return;

    if (this->discoveryCooldown <= 0.0)
    {
      DiscoverActorsAndProxies(_ecm);
      this->discoveryCooldown = 1.0;
    }
    else
    {
      this->discoveryCooldown -= dt;
    }

    for (auto &kv : this->actors)
    {
      auto taPtr = kv.second;
      if (!taPtr) continue;
      if (taPtr->actor == kNullEntity || taPtr->proxy == kNullEntity)
        continue;

      // 1) Pose actual del actor
      auto poseComp = _ecm.Component<components::Pose>(taPtr->actor);
      if (!poseComp)
        continue;
      const math::Pose3d pose = poseComp->Data();
      double yaw = YawOf(pose.Rot());

      // 2) Mueve el proxy delante del actor a altura del sensor
      math::Pose3d proxyPose = pose;
      proxyPose.Pos() += math::Vector3d(std::cos(yaw)*0.3, std::sin(yaw)*0.3, 0.0);
      proxyPose.Pos().Z() = this->sensorHeight;
      SetPoseCmd(_ecm, taPtr->proxy, proxyPose);

      // 3) Medidas del scan
      const double minR   = taPtr->minRange.load(std::memory_order_relaxed);
      const double leftR  = taPtr->leftMin.load(std::memory_order_relaxed);
      const double rightR = taPtr->rightMin.load(std::memory_order_relaxed);

      // 4) Corrección cinemática
      if (minR < this->minDistance)
      {
        int side = +1;
        if (rightR > leftR) side = -1;

        const double deltaYaw = this->turnRate * dt * static_cast<double>(side);
        const double lateral  = this->lateralSpeed * dt * static_cast<double>(side);

        yaw = WrapPi(yaw + deltaYaw);

        math::Vector3d rightVec(-std::sin(yaw), std::cos(yaw), 0.0);
        math::Pose3d corrected = pose;
        corrected.Pos() += rightVec * lateral;
        corrected.Rot() = math::Quaterniond(0, 0, yaw);

        SetPoseCmd(_ecm, taPtr->actor, corrected);
      }

      taPtr->lastPose = pose;
      taPtr->hasLastPose = true;
    }
  }

private:
  // Envía /world/<worldName>/set_pose con ignition::msgs::Pose
  void SetPoseCmd(EntityComponentManager &,
                  Entity e,
                  const ignition::math::Pose3d &p)
  {
    auto it = this->entityNames.find(e);
    if (it == this->entityNames.end())
      return;

    const std::string &entityName = it->second;

    ignition::msgs::Pose msg;
    msg.mutable_position()->set_x(p.Pos().X());
    msg.mutable_position()->set_y(p.Pos().Y());
    msg.mutable_position()->set_z(p.Pos().Z());

    ignition::math::Quaterniond q = p.Rot();
    msg.mutable_orientation()->set_w(q.W());
    msg.mutable_orientation()->set_x(q.X());
    msg.mutable_orientation()->set_y(q.Y());
    msg.mutable_orientation()->set_z(q.Z());

    msg.set_name(entityName);

    std::string service = "/world/" + this->worldName + "/set_pose";
    bool result{false};
    ignition::msgs::Boolean rep;

    (void)this->node.Request(service, msg, 100, rep, result);
  }

  // /world/<worldName>/model/<proxyName>/link/sensor_link/sensor/actor_front_ray/scan
  std::string MakeScanTopic(const std::string &proxyName) const
  {
    return "/world/" + this->worldName + "/model/" + proxyName
           + "/link/sensor_link/sensor/actor_front_ray/scan";
  }

  static std::vector<std::string> CollectSuffixes(
      const std::unordered_map<std::string, Entity> &actorsByName,
      const std::string &prefix)
  {
    std::vector<std::string> out;
    out.reserve(actorsByName.size());
    for (const auto &kv : actorsByName)
    {
      const auto &nm = kv.first;
      if (!StartsWith(nm, prefix)) continue;
      std::string sfx = nm.substr(prefix.size());
      out.push_back(sfx);
    }
    return out;
  }

  void DiscoverActorsAndProxies(EntityComponentManager &_ecm)
  {
    std::unordered_map<std::string, Entity> actorsByName;
    std::unordered_map<std::string, Entity> proxiesByName;

    _ecm.Each<components::Name>(
      [&](const Entity &_entity, const components::Name *_name)->bool
      {
        const std::string &nm = _name->Data();
        if (StartsWith(nm, this->actorPrefix))
          actorsByName[nm] = _entity;
        if (StartsWith(nm, this->proxyPrefix))
          proxiesByName[nm] = _entity;
        return true;
      });

    const auto suffixes = CollectSuffixes(actorsByName, this->actorPrefix);

    for (const auto &sfx : suffixes)
    {
      const std::string aName = this->actorPrefix + sfx;
      const std::string pName = this->proxyPrefix + sfx;

      if (!actorsByName.count(aName) || !proxiesByName.count(pName))
        continue;

      const Entity a = actorsByName[aName];
      const Entity p = proxiesByName[pName];

      auto it = this->actors.find(aName);
      if (it == this->actors.end())
      {
        auto taPtr = std::make_shared<TrackedActor>();
        taPtr->actor = a;
        taPtr->proxy = p;
        taPtr->actorName = aName;
        taPtr->proxyName = pName;
        taPtr->scanTopic = MakeScanTopic(pName);
        this->actors.emplace(aName, std::move(taPtr));
      }
      else
      {
        auto &taPtr = it->second;
        if (!taPtr) taPtr = std::make_shared<TrackedActor>();
        taPtr->actor = a;
        taPtr->proxy = p;
        taPtr->actorName = aName;
        taPtr->proxyName = pName;
        taPtr->scanTopic = MakeScanTopic(pName);
      }

      // Cachea nombres de entidades para el servicio set_pose
      this->entityNames[a] = aName;
      this->entityNames[p] = pName;
    }
  }

private:
  // Estado world / parámetros
  Entity worldEntity{kNullEntity};
  std::string worldName{"industrial-warehouse"}; // se sobreescribe si lo encuentra

  std::string actorPrefix{"actor_walking"};
  std::string proxyPrefix{"actor_sensor_proxy"};

  double minDistance{0.6};   // m
  double turnRate{1.0};      // rad/s
  double lateralSpeed{0.4};  // m/s
  double sensorHeight{1.1};  // m

  double discoveryCooldown{0.0};

  ignition::transport::Node node;

  // nombre del actor -> datos
  std::unordered_map<std::string, std::shared_ptr<TrackedActor>> actors;

  // cache: Entity -> nombre (para set_pose)
  std::unordered_map<Entity, std::string> entityNames;
};

IGNITION_ADD_PLUGIN(ActorAvoidance,
                    gazebo::System,
                    ActorAvoidance::ISystemConfigure,
                    ActorAvoidance::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ActorAvoidance, "actor_avoidance::World")