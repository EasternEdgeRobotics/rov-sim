/*
* rov-sim
* Copyright 2024 Eastern Edge Robotics
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
* Developer: Zaid Duraid
*/
#include "EER_Servo.hh"

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

class gz::sim::systems::ServoPrivate
{
  /// \brief Callback for velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnCmdVel(const msgs::Double &_msg);

  /// \brief Based on the joint name, get the joint entity and child link 
  /// entity and set the yaw boundries based on the user-specified yaw offsets.
  //  Note that "yaw" is used as the servo will rotate about the z-axis of the joint.
  /// \param[in]  _jointName the joint name of the servo joint
  /// \param[out] _jointEntity the joint entity of the servo joint
  /// \param[out] _linkEntity the link entity of the servo joint
  /// \param[out]  _yawBoundries the yaw boundries of the servo joint
  /// \param[in]  _maxYawOffsets the max yaw offsets of the servo joint
  /// \param[in]  _ecm the entity component manager
  /// \param[in]  _model the model entity
  /// \return true if the joint entity and link entity are found or are already initialized
  public: bool InitializeServo(const std::string &_jointName, Entity &_jointEntity, Entity &_linkEntity, EntityComponentManager &_ecm, Model &_model);

  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Servo joint name
  public: std::string servoJointName;
  
  /// \brief Servo joint entity
  public: Entity servoJointEntity{kNullEntity};

  /// \brief Servo link entity
  public: Entity servoLinkEntity{kNullEntity};

  /// \brief Commanded joint velocity
  public: double jointVelCmd{0.0};

  /// \brief mutex to protect jointVelCmd
  public: std::mutex jointVelCmdMutex;

  /// \brief Model interface
  public: Model model{kNullEntity};
};

//////////////////////////////////////////////////
Servo::Servo()
  : dataPtr(std::make_unique<ServoPrivate>())
{
}

//////////////////////////////////////////////////
void Servo::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "Servo plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // Subscribe to commands
  std::string topic;
  if ((!_sdf->HasElement("sub_topic")) && (!_sdf->HasElement("topic")))
  {
    topic = transport::TopicUtils::AsValidTopic(this->dataPtr->model.Name(_ecm) + "/servo/cmd_vel");
    if (topic.empty())
    {
      gzerr << "Failed to create topic for servo plugin with model ["
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
      gzerr << "Failed to create topic for servp for model ["
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

  if (_sdf->HasElement("joint"))
  {
    this->dataPtr->servoJointName = _sdf->Get<std::string>("joint");
  }
  else
  {
    gzerr << "Servo plugin requires a joint name to be specified. "
          << "Since none is specified, the plugin will not work for [" 
          << this->dataPtr->model.Name(_ecm) << "]." << std::endl;
    return;
  }

  this->dataPtr->node.Subscribe(topic,
    &ServoPrivate::OnCmdVel,
    this->dataPtr.get());

  gzmsg << "Servo subscribing to Double messages on [" << topic
        << "]" << std::endl;
}

//////////////////////////////////////////////////
void Servo::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("Servo::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  // Ensure the servo joint entity is initialized. If not, its child link and initalize the yaw boundries.
  if (!this->dataPtr->InitializeServo(this->dataPtr->servoJointName, this->dataPtr->servoJointEntity, this->dataPtr->servoLinkEntity, _ecm, this->dataPtr->model))
  {
    gzerr << "Failed to initialize servo. Servo plugin will not work for model [" 
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

  _ecm.SetComponentData<components::JointVelocityCmd>(this->dataPtr->servoJointEntity, {targetVel});
}

//////////////////////////////////////////////////
void ServoPrivate::OnCmdVel(const msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(this->jointVelCmdMutex);
  this->jointVelCmd = _msg.data();
}

//////////////////////////////////////////////////
bool ServoPrivate::InitializeServo(const std::string &_jointName, Entity &_jointEntity, Entity &_linkEntity, EntityComponentManager &_ecm, Model &_model)
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
    }
  }
  return true;
}

GZ_ADD_PLUGIN(Servo,
              System,
              Servo::ISystemConfigure,
              Servo::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(Servo,
                          "gz::sim::systems::Servo")