/**
 * @Author: Erchao Rong
 * @Date:   2023-04-24 16:20:46
 * @Last Modified by:   Erchao Rong
 * @Last Modified time: 2023-05-11 15:36:06
 */
/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "windPlugin.hh"




using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::windPluginPrivate
{
public:
  std::shared_ptr<ros::NodeHandle> rosNode;

  // Initialize the system
public:
  // void Load(const EntityComponentManager &_ecm,
  //           const sdf::ElementPtr &_sdf);
  void Load(const EntityComponentManager &_ecm,
            const sdf::ElementPtr &_sdf);

  /// \brief Compute lift and drag forces and update the corresponding
  /// components
  /// \param[in] _ecm Immutable reference to the EntityComponentManager
public:
  void Update(EntityComponentManager &_ecm);

public:
  void OnReceiveMessage(const gz::msgs::Vector3d& msg);
public:
  void endwind(const gz::msgs::Vector3d& msg);

  /// \brief Model interface
public:
  Model model{kNullEntity};

  /// \brief angle of attach when airfoil stalls  Reserved
public:
  double alphaStall = GZ_PI_2;

  /// \brief air density
  /// at 25 deg C it's about 1.1839 kg/m^3
  /// At 20 °C and 101.325 kPa, dry air has a density of 1.2041 kg/m3.
public:
  double rho = 1.2041;

  /// \brief effective planeform surface area
public:
  double area = 1.0;

  /// \brief center of pressure in link local coordinates with respect to the
  /// link's center of mass
public:
  gz::math::Vector3d cp = math::Vector3d::Zero;

  /// \brief Link entity targeted this plugin.
public:
  Entity linkEntity;

  /// \brief Set during Load to true if the configuration for the system is
  /// valid and the post-update can run
public:
  bool validConfig{false};

  /// \brief Copy of the sdf configuration used for this plugin
public:
  sdf::ElementPtr sdfConfig;

  /// \brief Initialization flag
public:
  bool initialized{false};

public:
  math::Vector3d windDirection;
  
public:
  gz::transport::Node::Publisher _force_pub;

public:
  gz::transport::Node::Publisher _moment_pub;



public:
  gz::transport::Node::Publisher _lin_vel_pub;

public:
  gz::transport::Node::Publisher _ang_vel_pub;

public:
  gz::transport::Node _node;

public:
  bool advertised{false};
  bool ifOmitMoment{false};
  bool ifOmitForce{false};
public:
  bool if_get_message{false};
public:
  bool if_end_wind{false};
};

void windPluginPrivate::OnReceiveMessage(const gz::msgs::Vector3d& msg)
{
    // 使用成员函数来获取向量的各个分量
    double x = msg.x();
    double y = msg.y();
    double z = msg.z();

    // 将收到的向量赋值给 windDirection
    this->windDirection = math::Vector3d(x, y, z);
    gzerr << "windDirection" << this->windDirection <<"\n";

    if_get_message=true;
    if_end_wind=false;
}
void windPluginPrivate::endwind(const gz::msgs::Vector3d& msg)
{

   if_end_wind=true;
   if_get_message=false;
}
//////////////////////////////////////////////////
void windPluginPrivate::Load(const EntityComponentManager &_ecm,
                                     const sdf::ElementPtr &_sdf)
{
  // this->alphaStall = _sdf->Get<double>("alpha_stall", this->alphaStall).first;

  this->rho = _sdf->Get<double>("air_density", this->rho).first;
  this->area = _sdf->Get<double>("area", this->area).first;

  this->cp = _sdf->Get<math::Vector3d>("cp", this->cp).first;

  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    auto linkName = elem->Get<std::string>();
    auto entities =
        entitiesFromScopedName(linkName, _ecm, this->model.Entity());
    gzerr << "Link with name[" << linkName <<"\n";
    if (entities.empty())
    {
      gzerr << "Link with name[" << linkName << "] not found. "
            << "The windPlugin will not generate forces\n";
      this->validConfig = false;
      return;
    }
    else if (entities.size() > 1)
    {
      gzerr << "Multiple link entities with name[" << linkName << "] found. "
            << "Please use unique name.\n";
    }

    this->linkEntity = *entities.begin();
    if (!_ecm.EntityHasComponentType(this->linkEntity,
                                     components::Link::typeId))
    {
      this->linkEntity = kNullEntity;
      gzerr << "Entity with name[" << linkName << "] is not a link\n";
      this->validConfig = false;
      return;
    }
  }
  else
  {
    gzerr << "The windPlugin system requires the 'link_name' parameter\n";
    this->validConfig = false;
    return;
  }

  // If we reached here, we have a valid configuration
  this->validConfig = true;

  // if(_sdf->HasElement(""))
  if (advertised)
  {
  }
  else
  {

    ifOmitMoment = _sdf->Get<bool>("if_omit_moment");
    ifOmitForce = _sdf->Get<bool>("if_omit_force");



    _node.Subscribe("/direction", &windPluginPrivate::OnReceiveMessage, this);
    _node.Subscribe("/end_wind", &windPluginPrivate::endwind, this);
    advertised = true;
  }
  // std::string actuator_topic = "/" + model_name + "/command/motor_speed";
  // _actuators_pub = _node.Advertise<gz::msgs::Actuators>(actuator_topic);
  // gzerr << force << "\n";
  // gzerr << moment << "\n";
}

//////////////////////////////////////////////////
// windPlugin::windPlugin(int argc, char** argv)
windPlugin::windPlugin()
    : System(), dataPtr(std::make_unique<windPluginPrivate>())
{

}

//////////////////////////////////////////////////
void windPluginPrivate::Update(EntityComponentManager &_ecm)
{

  math::Vector3d torqueI(0, 0, 0);
  math::Vector3d forceI(0,0,0);
  if(if_get_message){
    forceI.X()=windDirection.Z()*windDirection.X();
    forceI.Y()=windDirection.Z()*windDirection.Y();
    forceI.Z()=0;
    // if_get_message=false;
  }
  if(if_end_wind){
    forceI.X()=0;
    forceI.Y()=0;
    forceI.Z()=0; 
  }
  Link link(this->linkEntity);
  gzerr << "windDirection        " <<  sqrt(forceI.X()* forceI.X()+ forceI.Y()* forceI.Y()) <<"\n";
  link.AddWorldWrench(_ecm, forceI, torqueI);




}

//////////////////////////////////////////////////
void windPlugin::Configure(const Entity &_entity,
                                   const std::shared_ptr<const sdf::Element> &_sdf,
                                   EntityComponentManager &_ecm, EventManager &)
{
  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "The windPlugin system should be attached to a model entity. "
          << "Failed to initialize." << std::endl;
    return;
  }
  this->dataPtr->sdfConfig = _sdf->Clone();
}

//////////////////////////////////////////////////
void windPlugin::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  GZ_PROFILE("windPlugin::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  if (!this->dataPtr->initialized)
  {
    // We call Load here instead of Configure because we can't be guaranteed
    // that all entities have been created when Configure is called
    this->dataPtr->Load(_ecm, this->dataPtr->sdfConfig);
    this->dataPtr->initialized = true;

    if (this->dataPtr->validConfig)
    {
      Link link(this->dataPtr->linkEntity);
      link.EnableVelocityChecks(_ecm, true);
    }
  }

  if (_info.paused)
    return;

  // This is not an "else" because "initialized" can be set in the if block
  // above
  if (this->dataPtr->initialized && this->dataPtr->validConfig)
  {
    this->dataPtr->Update(_ecm);
  }
}

GZ_ADD_PLUGIN(windPlugin,
              System,
              windPlugin::ISystemConfigure,
              windPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(windPlugin, "gz::sim::systems::windPlugin")

// TODO(CH3): Deprecated, remove on version 8
GZ_ADD_PLUGIN_ALIAS(windPlugin, "ignition::gazebo::systems::windPlugin")
