/**
 * @Author: Erchao Rong
 * @Date:   2023-04-24 18:14:54
 * @Last Modified by:   Erchao Rong
 * @Last Modified time: 2023-04-25 14:53:12
 */

/**
 * S : Area of the wing
 * rho : the air density 
 * link_name : what link the aero force added on
 * **/

#ifndef GZ_SIM_SYSTEMS_LIFT_DRAG_HH_
#define GZ_SIM_SYSTEMS_LIFT_DRAG_HH_
#include <algorithm>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

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
#include <memory>
#include <gz/sim/System.hh>
#include <ros/ros.h>
#include "gz/transport.hh"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  constexpr double air_density_in_model = 1.184;
  constexpr double area_in_model = 0.1955;
  
  // Forward declaration
  class windPluginPrivate;

  /// \brief The windPlugin system computes lift and drag forces enabling
  /// simulation of aerodynamic robots.
  ///
  class windPlugin
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    
    /// \brief Constructor
    // public: windPlugin();
    public: windPlugin() ;//= default;
    // public: windPlugin(int argc, char** argv);
    /// \brief Destructor
    public: ~windPlugin() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    /// Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) final;
    // public: static void OnReceiveMessage(const gz::msgs::Vector3d& msg);

    /// \brief Private data pointer
    private: std::unique_ptr<windPluginPrivate> dataPtr;
    // private: std::shared_ptr<ros::NodeHandle> rosNode;
    // private: ros::NodeHandle rosNode;

    // 声明 goalSub 订阅器
    // private: ros::Subscriber goalSub;
  };
  }
}
}
}

#endif