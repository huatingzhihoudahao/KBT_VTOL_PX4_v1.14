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

#include "AerodynamicsPlugin.hh"
#include "Zhang/Aerodynamics.h"
#include <algorithm>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/math/Quaternion.hh>
#include <sdf/Element.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/components/Pose.hh"
// #include "gz/msgs/wrench.pb.h"
#include "gz/msgs.hh"
#include "gz/transport.hh"
// #include "gz/"
// #include "gz/sim/math/"
// #include
// #define <EIGEN3/
// #include "gz/math/eigen3.hh"
// #include "eigen3/Eigen/Geometry"
// #include "eigen3/Eigen/"
// #include <ros/ros.h>
// #include <std_msgs/Float64.h>


#define align_by_gazebo_orientation

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::AerodynamicsPluginPrivate
{
  
  // Initialize the system
public:
  void Load(const EntityComponentManager &_ecm,
            const sdf::ElementPtr &_sdf);


  /// \brief Compute lift and drag forces and update the corresponding
  /// components
  /// \param[in] _ecm Immutable reference to the EntityComponentManager
public:
  void Update(EntityComponentManager &_ecm);

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
  std::string _topic_force;

public:
  std::string _topic_moment;

public:
  gz::transport::Node::Publisher _force_pub,v_pub,q_pub,v2_pub,q2_pub,p_pub;
  gz::transport::Node::Publisher alpha_beta_pub;
  

public:
  gz::transport::Node::Publisher _moment_pub;

public:
  gz::transport::Node::Publisher _lin_vel_pub;

public:
  gz::transport::Node::Publisher _ang_vel_pub;

public:
  gz::transport::Node _node;
//   ros::NodeHandle nh;
//   ros::Publisher force_pub;
public: 
  double q_from_imu_w ;
  double q_from_imu_x ;
  double q_from_imu_y ;
  double q_from_imu_z ;

  double vx_from_odom ;
  double vy_from_odom ;
  double vz_from_odom ;

  bool get_q_from_imu{false};
  bool get_v_from_odom{false};
public:
  bool advertised{false};
  bool ifOmitMoment{false};
  bool ifOmitForce{false};
public:
  void OnReceiveMessage(const gz::msgs::Quaternion& msg);
  void OnReceiveMessage2(const gz::msgs::Vector3d& msg);
};
void AerodynamicsPluginPrivate::OnReceiveMessage(const gz::msgs::Quaternion& msg){
#ifdef align_by_gazebo_orientation
#else 
    get_q_from_imu = true;
#endif
    q_from_imu_w = msg.w();
    q_from_imu_x= msg.x();
    q_from_imu_y = msg.y();
    q_from_imu_z = msg.z();
}
void AerodynamicsPluginPrivate::OnReceiveMessage2(const gz::msgs::Vector3d& msg){
#ifdef align_by_gazebo_orientation
#else 
    get_v_from_odom = true;
#endif

    vx_from_odom = msg.x();
    vy_from_odom= msg.y();
    vz_from_odom = msg.z();

}

//////////////////////////////////////////////////
void AerodynamicsPluginPrivate::Load(const EntityComponentManager &_ecm,
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

    if (entities.empty())
    {
      gzerr << "Link with name[" << linkName << "] not found. "
            << "The AerodynamicsPlugin will not generate forces\n";
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
    gzerr << "The AerodynamicsPlugin system requires the 'link_name' parameter\n";
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
        // 初始化 ROS 节点
    // if (!ros::isInitialized()) {
    //   int argc = 0;
    //   char **argv = nullptr;
    //   ros::init(argc, argv, "aerodynamics_plugin");
    // }
  // 创建 ROS Publisher
    // force_pub = nh.advertise<std_msgs::Float64MultiArray>("aerodynamics/force", 10);

    auto force = _sdf->Get<std::string>("topic_force");
    auto moment = _sdf->Get<std::string>("topic_moment");
    ifOmitMoment = _sdf->Get<bool>("if_omit_moment");
    ifOmitForce = _sdf->Get<bool>("if_omit_force");
    if (force.size() == 0 && moment.size() == 0)
      gzerr << "The AerodynamicsPlugin system requires the 'topic_force' 'topic_moment' parameter\n";

    // this->_topic_force = force;
    // this->_topic_moment = moment;

    _force_pub = _node.Advertise<gz::msgs::Vector3d>("/" + model.Name(_ecm) + "/" + force);
    p_pub= _node.Advertise<gz::msgs::Vector3d>("/" + model.Name(_ecm) + "/" + "plugin_p");
    v_pub= _node.Advertise<gz::msgs::Vector3d>("/" + model.Name(_ecm) + "/" + "plugin_v");
    q_pub= _node.Advertise<gz::msgs::Quaternion>("/" + model.Name(_ecm) + "/" + "plugin_q");
    q2_pub= _node.Advertise<gz::msgs::Quaternion>("/" + model.Name(_ecm) + "/" + "plugin_q2");
    v2_pub= _node.Advertise<gz::msgs::Vector3d>("/" + model.Name(_ecm) + "/" + "plugin_v2");
    alpha_beta_pub = _node.Advertise<gz::msgs::Vector2d>("/" + model.Name(_ecm) + "/" + "alpha_beta");
    _moment_pub = _node.Advertise<gz::msgs::Vector3d>("/" + model.Name(_ecm) + "/" + "moment");
    _lin_vel_pub = _node.Advertise<gz::msgs::Vector3d>("/" + model.Name(_ecm) + "/" + "lin_vel");
    _ang_vel_pub = _node.Advertise<gz::msgs::Vector3d>("/" + model.Name(_ecm) + "/" + "ang_vel");
    _node.Subscribe("/q_from_ros_imu", &AerodynamicsPluginPrivate::OnReceiveMessage,this);
    _node.Subscribe("/v_from_ros_odom", &AerodynamicsPluginPrivate::OnReceiveMessage2,this);
    advertised = true;
  }
  // std::string actuator_topic = "/" + model_name + "/command/motor_speed";
  // _actuators_pub = _node.Advertise<gz::msgs::Actuators>(actuator_topic);
  // gzerr << force << "\n";
  // gzerr << moment << "\n";
}

//////////////////////////////////////////////////
AerodynamicsPlugin::AerodynamicsPlugin()
    : System(), dataPtr(std::make_unique<AerodynamicsPluginPrivate>())
{
}

//////////////////////////////////////////////////
void AerodynamicsPluginPrivate::Update(EntityComponentManager &_ecm)
{
  GZ_PROFILE("AerodynamicsPluginPrivate::Update");
  // get linear velocity at cp in world frame
  const auto worldLinVel =
      _ecm.Component<components::WorldLinearVelocity>(this->linkEntity);
  const auto worldAngVel =
      _ecm.Component<components::WorldAngularVelocity>(this->linkEntity);
  const auto worldPose =
      _ecm.Component<components::WorldPose>(this->linkEntity);

  if (!worldLinVel || !worldAngVel || !worldPose)
    return;

  const auto &pose = worldPose->Data();//这里应该是重心
  const auto cpWorld = pose.Rot().RotateVector(this->cp);
  //气动中心是0   应该cpworld和pose是一样的
//示例
    //  ignition::math::Pose3d framePose(_frame->WorldPose().Pos(),\
    //  ignition::math::Quaterniond(0.0, 0.0,_frame->WorldPose().Rot().Yaw()));
  // std::cout<<"pose.Rot().w()    "<<pose.Rot().W()<<std::endl;

  gz::msgs::Vector3d p_msg;
  p_msg.set_x(pose.X());
  p_msg.set_y(pose.Y());
  p_msg.set_z(pose.Z());  
  p_pub.Publish(p_msg);
  math::Vector3d vel;
  if(!get_v_from_odom){ 
   vel = worldLinVel->Data() ;//+ worldAngVel->Data().Cross(cpWorld);
  }
  else{
    vel.X()=(vx_from_odom);
    vel.Y()=(vy_from_odom);
    vel.Z()=(vz_from_odom);
  }
  //vel就是worldlinvel
  if (vel.Length() <= 0.01)
    const auto velI = math::Vector3d::Zero;
  // else
  //   const auto velI = vel.Normalized();
  math::Vector3d velB_FLU,velB;
  auto Q_frd2flu = math::Quaternion<double>(0, 1.0, 0, 0);
  if(!get_q_from_imu){
      velB_FLU = pose.Rot().RotateVectorReverse(vel);

      velB = Q_frd2flu.RotateVectorReverse(velB_FLU);
  }
  else{
    math::Quaternion q_from_imu(q_from_imu_w, q_from_imu_x, q_from_imu_y, q_from_imu_z);
    
    velB = q_from_imu.RotateVectorReverse(vel);
  }

  
//明天弄一下姿态发出去  现在确信  气动中心是0   应该cpworld和pose是一样的  但vel还是和预期的不一样 
  // TODO wrapper to use
  // auto Q_frd2flu = math::Quaternion<double>(0, 1.0, 0, 0);

  // auto velB = Q_frd2flu.RotateVectorReverse(velB_FLU);
  // velB.Eog
  double vB_double[3] = {velB.X(), velB.Y(), velB.Z()};
  double fB_double[3] = {0};
  double mB_double[3] = {0};
  double alpha = 0;
  double beta = 0;
  Aerodynamics(vB_double, fB_double, mB_double,
               &alpha, &beta);//这里fB_double好像是力
  gz::msgs::Vector2d alpha_beta_msg;
  gz::msgs::Vector3d v_msg,v2_msg;
  gz::msgs::Quaternion q_msg,q2_msg;
  math::Quaternion q1(pose.Rot().W(),pose.Rot().X(),pose.Rot().Y(),pose.Rot().Z());
#ifdef align_by_gazebo_orientation
    math::Quaternion q2(0.0, 1.0, 0.0, 0.0);
    math::Quaternion q_=q1*q2;
    q_msg.set_w(q_.W());
    q_msg.set_x(q_.X());
    q_msg.set_y(q_.Y());
    q_msg.set_z(q_.Z());
#else
    math::Quaternion q_from_imu_tmp(q_from_imu_w, q_from_imu_x, q_from_imu_y, q_from_imu_z);
    q_msg.set_w(q_from_imu_tmp.W());
    q_msg.set_x(q_from_imu_tmp.X());
    q_msg.set_y(q_from_imu_tmp.Y());
    q_msg.set_z(q_from_imu_tmp.Z());
#endif
  v_msg.set_x(velB.X());
  v_msg.set_y(velB.Y());
  v_msg.set_z(velB.Z());  

  v2_msg.set_x(vel.X());
  v2_msg.set_y(vel.Y());
  v2_msg.set_z(vel.Z());  



  q2_msg.set_w(q1.W());
  q2_msg.set_x(q1.X());
  q2_msg.set_y(q1.Y());
  q2_msg.set_z(q1.Z());

  // v_msg.set_x(vel.X());
  // v_msg.set_y(vel.Y());
  // v_msg.set_z(vel.Z());  
  alpha_beta_msg.set_x(alpha);
  alpha_beta_msg.set_y(beta);
  
  // std::cout<<"alpha    "<<alpha<<std::endl;
  // std::cout<<"          beta    "<<beta<<std::endl;
  // spanwiseI used to be momentDirection
  // math::Vector3d moment = cm * q * this->area * spanwiseI;
  // force and torque about cg in world frame
  math::Vector3d forceB(fB_double[0], fB_double[1], fB_double[2]);
  math::Vector3d torqueB(mB_double[0], mB_double[1], mB_double[2]);
   gz::msgs::Vector3d force_msg; 
  force_msg.set_x(forceB.X());
  force_msg.set_y(forceB.Y());
  force_msg.set_z(forceB.Z());
  if (this->ifOmitForce)
    forceB = math::Vector3d::Zero;
  else
    forceB = Q_frd2flu.RotateVector(forceB);

  if (this->ifOmitMoment)
    torqueB = math::Vector3d::Zero;
  else
    torqueB = Q_frd2flu.RotateVector(torqueB);

  // Correct for nan or inf
  forceB.Correct();
  // this->cp.Correct();
  torqueB.Correct();


  torqueB.X() = 0;//0.5*torqueB.X();//0;
  torqueB.Y() = 0;
  // torqueB.Z() = 0;


  // torqueB.Y() = 0;
  // We want to apply the force at cp. The old AerodynamicsPlugin plugin did the
  // following:
  //     this->link->AddForceAtRelativePosition(force, this->cp);
  // The documentation of AddForceAtRelativePosition says:
  //> Add a force (in world frame coordinates) to the body at a
  //> position relative to the center of mass which is expressed in the
  //> link's own frame of reference.
  // But it appears that 'cp' is specified in the link frame so it probably
  // should have been
  //     this->link->AddForceAtRelativePosition(
  //         force, this->cp - this->link->GetInertial()->CoG());
  //
  // \todo(addisu) Create a convenient API for applying forces at offset
  // positions


  // std::cout << "FRD Torque: " << torqueB.X() << ", " << torqueB.Y() << ", " << torqueB.Z() << std::endl;
  auto forceI = pose.Rot().RotateVector(forceB);
  auto torqueI = pose.Rot().RotateVector(torqueB);
  // torqueI.X() = 0;
  // torqueI.Y() = 0;
  // torqueI.Z() = 0;



  
  // torqueI.Z() = 0;
  // torqueI.X() = 0;
  // const auto totalTorque = torque + cpWorld.Cross(force);
  Link link(this->linkEntity);

  link.AddWorldWrench(_ecm, forceI, torqueI);

  // gz::msgs::Actuators msg;
  // msg.set


  gz::msgs::Vector3d torque_msg;

  torque_msg.set_x(torqueB.X());
  torque_msg.set_y(torqueB.Y());
  torque_msg.set_z(torqueB.Z());

  // gz::msgs::Ve
  auto vel_msg = gz::msgs::Convert(worldLinVel->Data());
  auto ang_msg = gz::msgs::Convert(worldAngVel->Data());

  // testing velB
  //  force_msg.set_x(velB.X());
  //  force_msg.set_y(velB.Y());
  //  force_msg.set_z(velB.Z());

  _force_pub.Publish(force_msg);
  alpha_beta_pub.Publish(alpha_beta_msg);
  v_pub.Publish(v_msg);
  v2_pub.Publish(v2_msg);
  q_pub.Publish(q_msg);
  q2_pub.Publish(q2_msg);
  _moment_pub.Publish(torque_msg);
  _lin_vel_pub.Publish(vel_msg);
  _ang_vel_pub.Publish(ang_msg);
  // force_msg.set
  // gz::msgs::Wrench wrenchB;
  // wrenchB.set_allocated_force(&forceB);
  // wrenchB.set_allocated_torque(&torqueB);
  // Debug
  // auto linkName = _ecm.Component<components::Name>(this->linkEntity)->Data();
  // gzdbg << "=============================\n";
  // gzdbg << "Link: [" << linkName << "] pose: [" << pose
  //        << "] dynamic pressure: [" << q << "]\n";
  // gzdbg << "spd: [" << vel.Length() << "] vel: [" << vel << "]\n";
  // gzdbg << "LD plane spd: [" << velInLDPlane.Length() << "] vel : ["
  //        << velInLDPlane << "]\n";
  // gzdbg << "forward (inertial): " << forwardI << "\n";
  // gzdbg << "upward (inertial): " << upwardI << "\n";
  // gzdbg << "q: " << q << "\n";
  // gzdbg << "cl: " << cl << "\n";
  // gzdbg << "lift dir (inertial): " << liftI << "\n";
  // gzdbg << "Span direction (normal to LD plane): " << spanwiseI << "\n";
  // gzdbg << "sweep: " << sweep << "\n";
  // gzdbg << "alpha: " << alpha << "\n";
  // gzdbg << "lift: " << lift << "\n";
  // gzdbg << "drag: " << drag << " cd: " << cd << " cda: "
  //        << this->cda << "\n";
  // gzdbg << "moment: " << moment << "\n";
  // gzdbg << "velB: " << velB << "\n";
  // gzdbg << "force: " << forceB << "\n";
  // gzdbg << "torque: " << torqueB << "\n";
  // gzdbg << "totalTorque: " << totalTorque << "\n";
}

//////////////////////////////////////////////////
void AerodynamicsPlugin::Configure(const Entity &_entity,
                                   const std::shared_ptr<const sdf::Element> &_sdf,
                                   EntityComponentManager &_ecm, EventManager &)
{
  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "The AerodynamicsPlugin system should be attached to a model entity. "
          << "Failed to initialize." << std::endl;
    return;
  }
  this->dataPtr->sdfConfig = _sdf->Clone();
}

//////////////////////////////////////////////////
void AerodynamicsPlugin::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  GZ_PROFILE("AerodynamicsPlugin::PreUpdate");

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

GZ_ADD_PLUGIN(AerodynamicsPlugin,
              System,
              AerodynamicsPlugin::ISystemConfigure,
              AerodynamicsPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(AerodynamicsPlugin, "gz::sim::systems::AerodynamicsPlugin")

// TODO(CH3): Deprecated, remove on version 8
GZ_ADD_PLUGIN_ALIAS(AerodynamicsPlugin, "ignition::gazebo::systems::AerodynamicsPlugin")
