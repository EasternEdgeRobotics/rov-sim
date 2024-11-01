/*
* rov-sim
* Copyright (C) 2024  Eastern Edge Robotics
* 
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Affero General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Affero General Public License for more details.
* 
* You should have received a copy of the GNU Affero General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
* Developer: Zaid Duraid
*/
#include "EER_Claws.hh"

#include <gz/msgs/double.pb.h>

#include <string>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/Pose.hh"

#include "gz/sim/Model.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::ClawsPrivate
{
  /// \brief Callback for velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnCmdVel(const msgs::Double &_msg);

  /// \brief Based on the joint name, get the joint entity and child link 
  /// entity and set the yaw boundries based on the user-specified yaw offsets of the claws
  /// \param[in]  _jointName the joint name of the claw join
  /// \param[out] _jointEntity the joint entity of the claw joint
  /// \param[out] _linkEntity the link entity of the claw joint
  /// \param[out]  _yawBoundries the yaw boundries of the claw joint
  /// \param[in]  _maxYawOffsets the max yaw offsets of the claw joint
  /// \param[in]  _ecm the entity component manager
  /// \param[in]  _model the model entity
  /// \return true if the joint entity and link entity are found or are already initialized
  public: bool InitializeClaw(const std::string &_jointName, Entity &_jointEntity, Entity &_linkEntity, std::array<double, 2> &_yawBoundries, const std::array<double, 2> &_maxYawOffsets, EntityComponentManager &_ecm, Model &_model);

  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Right claw joint name
  public: std::string rightClawJointName;
  
  /// \brief Left claw joint name
  public: std::string leftClawJointName;

  /// \brief Right claw joint entity
  public: Entity rightClawJointEntity{kNullEntity};

  /// \brief Left claw joint entity
  public: Entity leftClawJointEntity{kNullEntity};

  /// \brief Right claw link entity
  public: Entity rightClawLinkEntity{kNullEntity};

  /// \brief Left claw link entity
  public: Entity leftClawLinkEntity{kNullEntity};

  /// \brief Right claw yaw boundries
  public: std::array<double, 2> rightClawYawBoundries;

  /// \brief left claw yaw boundries
  public: std::array<double, 2> leftClawYawBoundries;

  /// \brief Relative yaw boundries for the right claw from the initial yaw value
  public: std::array<double, 2> rightClawMaxYawOffsets;

  /// \brief Relative yaw boundries for the left claw from the initial yaw value
  public: std::array<double, 2> leftClawMaxYawOffsets;

  /// \brief Commanded joint velocity
  public: double jointVelCmd{0.0};

  /// \brief mutex to protect jointVelCmd
  public: std::mutex jointVelCmdMutex;

  /// \brief Model interface
  public: Model model{kNullEntity};
};

//////////////////////////////////////////////////
Claws::Claws()
  : dataPtr(std::make_unique<ClawsPrivate>())
{
}

//////////////////////////////////////////////////
void Claws::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "Claws plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // Get params from SDF
  auto rightClawTag = _sdf->FindElement("right_claw");
  auto leftClawTag = _sdf->FindElement("left_claw");

  if (rightClawTag != nullptr && leftClawTag != nullptr)
  {
    if (!rightClawTag->HasElement("joint") || !rightClawTag->HasElement("max_inward_rotation") || !rightClawTag->HasElement("max_outward_rotation")
      || !leftClawTag->HasElement("joint") || !leftClawTag->HasElement("max_inward_rotation") || !leftClawTag->HasElement("max_outward_rotation"))
    {
      gzerr << "A joint, max inward rotation, and max outward rotation must be defined for each claw in the Claws plugin!\n" << std::endl;
      return;
    }
    else 
    {
      // Grab all the necessary parameters from the SDF
      this->dataPtr->rightClawJointName = rightClawTag->FindElement("joint")->Get<std::string>();
      this->dataPtr->leftClawJointName = leftClawTag->FindElement("joint")->Get<std::string>();
      this->dataPtr->rightClawMaxYawOffsets[0] = rightClawTag->FindElement("max_inward_rotation")->Get<double>();
      this->dataPtr->rightClawMaxYawOffsets[1] = rightClawTag->FindElement("max_outward_rotation")->Get<double>();
      this->dataPtr->leftClawMaxYawOffsets[0] = leftClawTag->FindElement("max_inward_rotation")->Get<double>();
      this->dataPtr->leftClawMaxYawOffsets[1] = leftClawTag->FindElement("max_outward_rotation")->Get<double>();
    }
  }
  else
  {
    gzerr << "You must specify both a right claw tag and a left claw tag for the claws plugin! " 
          << "For each tag, specity <joint>, <max_inward_rotation>, and <max_outward_rotation>. " 
          << "Claws plugin will not work for [" << this->dataPtr->model.Name(_ecm) << "]." << std::endl;
    return;
  }

  // Subscribe to commands
  std::string topic;
  if ((!_sdf->HasElement("sub_topic")) && (!_sdf->HasElement("topic")))
  {
    topic = transport::TopicUtils::AsValidTopic(this->dataPtr->model.Name(_ecm) + "/claws/cmd_vel");
    if (topic.empty())
    {
      gzerr << "Failed to create topic for claws plugin with model ["
            << this->dataPtr->model.Name(_ecm)
            << "]" << std::endl;
      return;
    }
  }
  if (_sdf->HasElement("sub_topic"))
  {
    topic = transport::TopicUtils::AsValidTopic("/model/" +
      this->dataPtr->model.Name(_ecm) + "/" +
        _sdf->Get<std::string>("sub_topic"));

    if (topic.empty())
    {
      gzerr << "Failed to create topic for claws for model ["
             << this->dataPtr->model.Name(_ecm) << "]" << std::endl;
      return;
    }
  }
  if (_sdf->HasElement("topic"))
  {
    topic = transport::TopicUtils::AsValidTopic(
        _sdf->Get<std::string>("topic"));

    if (topic.empty())
    {
      gzerr << "Failed to create topic [" << _sdf->Get<std::string>("topic")
             << "]" << " for model [" << this->dataPtr->model.Name(_ecm)
             << "]" << std::endl;
      return;
    }
  }

  this->dataPtr->node.Subscribe(topic,
    &ClawsPrivate::OnCmdVel,
    this->dataPtr.get());

  gzmsg << "Claws subscribing to Double messages on [" << topic
        << "]" << std::endl;
}

//////////////////////////////////////////////////
void Claws::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("Claws::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  // Ensure right claw and left claw joint entities are initialized. If not, grab their child links and initalize the yaw boundries.
  if (!this->dataPtr->InitializeClaw(this->dataPtr->rightClawJointName, this->dataPtr->rightClawJointEntity, this->dataPtr->rightClawLinkEntity, this->dataPtr->rightClawYawBoundries, this->dataPtr->rightClawMaxYawOffsets, _ecm, this->dataPtr->model) ||
      !this->dataPtr->InitializeClaw(this->dataPtr->leftClawJointName, this->dataPtr->leftClawJointEntity, this->dataPtr->leftClawLinkEntity, this->dataPtr->leftClawYawBoundries, this->dataPtr->leftClawMaxYawOffsets, _ecm, this->dataPtr->model))
  {
    gzwarn << "Failed to initialize claws. Claws plugin will not work for model [" 
    << this->dataPtr->model.Name(_ecm) << "]."
    << std::endl;
    return;
  }

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  double targetVel;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->jointVelCmdMutex);
    targetVel = this->dataPtr->jointVelCmd;
  }

  auto rightClawYaw = _ecm.Component<components::Pose>(this->dataPtr->rightClawLinkEntity)->Data().Yaw();
  auto leftClawYaw = _ecm.Component<components::Pose>(this->dataPtr->leftClawLinkEntity)->Data().Yaw();

  // Consider the positive values of the right and left claw yaw
  if (rightClawYaw < 0) rightClawYaw += 2*M_PI;
  if (leftClawYaw < 0) leftClawYaw += 2*M_PI;

  double rightClawAppliedVel = 0;
  double leftClawAppliedVel = 0;

  auto isWithinBounds = [](double value, double upperBound, double lowerBound) -> bool
  {
    if (upperBound < lowerBound) return (value < upperBound || value > lowerBound);
    else return (value > lowerBound && value < upperBound);
  };

  if (isWithinBounds(rightClawYaw, this->dataPtr->rightClawYawBoundries[0], this->dataPtr->rightClawYawBoundries[1]))
  {
    rightClawAppliedVel = -targetVel;
  } 
  else 
  {
    if (std::abs(rightClawYaw - this->dataPtr->rightClawYawBoundries[0]) < std::abs(this->dataPtr->rightClawYawBoundries[1] - rightClawYaw))
    {
      rightClawAppliedVel = -1.0;
    }
    else rightClawAppliedVel = 1.0;
  }

  if (isWithinBounds(leftClawYaw, this->dataPtr->leftClawYawBoundries[0], this->dataPtr->leftClawYawBoundries[1]))
  {
    leftClawAppliedVel = targetVel;
  } 
  else 
  {
    if (std::abs(leftClawYaw - this->dataPtr->leftClawYawBoundries[0]) < std::abs(this->dataPtr->leftClawYawBoundries[1] - leftClawYaw))
    {
      leftClawAppliedVel = -1.0;
    } else leftClawAppliedVel = 1.0;
  }
  
  _ecm.SetComponentData<components::JointVelocityCmd>(this->dataPtr->rightClawJointEntity, {rightClawAppliedVel});
  _ecm.SetComponentData<components::JointVelocityCmd>(this->dataPtr->leftClawJointEntity, {leftClawAppliedVel});
}

//////////////////////////////////////////////////
void ClawsPrivate::OnCmdVel(const msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(this->jointVelCmdMutex);
  this->jointVelCmd = _msg.data();
}

//////////////////////////////////////////////////
bool ClawsPrivate::InitializeClaw(const std::string &_jointName, Entity &_jointEntity, Entity &_linkEntity, std::array<double, 2> &_yawBoundries, const std::array<double, 2> &_maxYawOffsets, EntityComponentManager &_ecm, Model &_model)
{
  if (_jointEntity == kNullEntity)
  {
    _jointEntity = _model.JointByName(_ecm, _jointName);
    if (_jointEntity == kNullEntity)
    {
      gzwarn << "Failed to find joint [" << _jointName << "]" << std::endl;
      return false;
    }
    if (_linkEntity == kNullEntity) 
    {  
      auto linkEntityName = _ecm.Component<components::ChildLinkName>(_jointEntity);
      _linkEntity = _model.LinkByName(_ecm, linkEntityName->Data());
      if (_linkEntity == kNullEntity)
      {
        // Really, this should never happen. Gazebo should always have a child link entity for each joint.
        gzwarn << "Failed to find child link [" << linkEntityName->Data() << "]" << std::endl;
        return false;
      }
      // Calculate the yaw boundries based on the inital yaw value and the max yaw offsets. 

      // Element 0 is inward for right claw, outward for left claw
      _yawBoundries[0] = _ecm.Component<components::Pose>(_linkEntity)->Data().Yaw() + _maxYawOffsets[0]; 
      while (_yawBoundries[0] > M_PI*2) _yawBoundries[0] -= M_PI*2;

      // Element 1 is outward for right claw, inward for left claw
      _yawBoundries[1] = _ecm.Component<components::Pose>(_linkEntity)->Data().Yaw() - _maxYawOffsets[1];
      while (_yawBoundries[1] < 0) _yawBoundries[1] += M_PI*2;
    }
  }
  return true;
}

GZ_ADD_PLUGIN(Claws,
                    System,
                    Claws::ISystemConfigure,
                    Claws::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(Claws,
                          "gz::sim::systems::Claws")