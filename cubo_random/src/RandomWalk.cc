#include <random>
#include <string>
#include <chrono>   // para std::chrono::duration
#include <cmath>    // para std::hypot

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Types.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/plugin/Register.hh>

using namespace ignition;
using namespace gazebo;

class RandomWalk :
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
    this->model = Model(_entity);
    if (!this->model.Valid(_ecm))
      return;

    if (_sdf && _sdf->HasElement("speed"))
      this->speed = _sdf->Get<double>("speed");
    if (_sdf && _sdf->HasElement("change_period"))
      this->changePeriod = _sdf->Get<double>("change_period");

    std::random_device rd;
    rng.seed(rd());
    dist = std::uniform_real_distribution<double>(-1.0, 1.0);

    PickNewDirection();
  }

  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;

    double now = std::chrono::duration<double>(_info.simTime).count();
    if (now - this->lastChange >= this->changePeriod) {
      PickNewDirection();
      this->lastChange = now;
    }

    math::Vector3d v(this->dirX * this->speed,
                     this->dirY * this->speed,
                     0.0);

    auto entity = this->model.Entity();
    auto comp = _ecm.Component<components::LinearVelocityCmd>(entity);
    if (comp) {
      *comp = components::LinearVelocityCmd(v);
    } else {
      _ecm.CreateComponent(entity, components::LinearVelocityCmd(v));
    }
  }

private:
  void PickNewDirection()
  {
    double x = dist(rng);
    double y = dist(rng);
    double n = std::hypot(x, y);
    if (n < 1e-6) { x = 1.0; y = 0.0; n = 1.0; }
    this->dirX = x / n;
    this->dirY = y / n;
  }

  Model model{kNullEntity};
  double speed{1.0};           // m/s
  double changePeriod{2.0};    // s
  double lastChange{0.0};

  // >>> Estas dos variables faltaban:
  double dirX{1.0};
  double dirY{0.0};
  // <<<

  std::mt19937 rng;
  std::uniform_real_distribution<double> dist;
};

IGNITION_ADD_PLUGIN(RandomWalk,
                    gazebo::System,
                    RandomWalk::ISystemConfigure,
                    RandomWalk::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(RandomWalk, "random_walk::Model")
