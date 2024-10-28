/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
 * 
 * Modifications Copyright (C) 2024 Eastern Edge Robotics
 * Modifications by Zaid Duraid 
 * Licensed under the MIT License.
 *
 * Description of modifications:
 * New plugin operation mode (ESCCmd) was added to allow the user to control the thruster using ESC input values (0-255) instead of force or angular velocity commands.
 */
#include <cstdlib>
#include <gz/transport/TopicUtils.hh>
#include <memory>
#include <mutex>
#include <limits>
#include <string>
#include <iostream>
#include <fstream>
#include <array>

#include <gz/msgs/double.pb.h>

#include <gz/math/Helpers.hh>
#include <gz/math/PID.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <gz/plugin/Register.hh>

#include <gz/transport/Node.hh>

#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/BatterySoC.hh"
#include "gz/sim/components/BatteryPowerLoad.hh"
#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

#include "EER_Thruster.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::ThrusterPrivateData
{
  /// \brief The mode of operation
  public: enum OperationMode {
    /// \brief Takes in a force commmand and spins the propeller at an
    /// appropriate rate.
    ForceCmd = 0,
    /// \brief Takes in angular velocity commands in radians per second and
    /// calculates the appropriate force.
    AngVelCmd,
    /// \brief Takes in a uint8 command and internally converts it a PWM
    /// input value from 1100 to 1900 microseconds. The PWM value is then 
    /// converted to thrust based on the bheavior BlueRobotics T200 thruster.
    /// See https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/ 
    ESCCmd
  } opmode = OperationMode::ESCCmd;

  /// \brief Mutex for read/write access to class
  public: std::mutex mtx;

  /// \brief Thrust output by propeller in N
  public: double thrust = 0.0;

  /// \brief Desired propeller angular velocity in rad / s
  public: double propellerAngVel = 0.0;

  /// \brief Enabled or not
  public: bool enabled = true;

  /// \brief Model entity
  public: Entity modelEntity;

  /// \brief The link entity which will spin
  public: Entity linkEntity;

  /// \brief Battery consumer entity
  public: Entity consumerEntity;

  /// \brief Axis along which the propeller spins. Expressed in the joint
  /// frame. Assume this doesn't change during simulation.
  public: math::Vector3d jointAxis;

  /// \brief Joint pose in the child link frame. Assume this doesn't change
  /// during the simulation.
  public: math::Pose3d jointPose;

  /// \brief Propeller joint entity
  public: Entity jointEntity;

  /// \brief Gazebo node for handling transport
  public: transport::Node node;

  /// \brief Publisher for feedback of data
  public: transport::Node::Publisher pub;

  /// \brief Second publisher for feedback of data
  public: transport::Node::Publisher pub2;

  /// \brief The PID which controls the propeller. This isn't used if
  /// velocityControl is true.
  public: math::PID propellerController;

  /// \brief Velocity Control mode - this disables the propellerController
  /// and writes the angular velocity directly to the joint. default: false
  public: bool velocityControl = false;

  /// \brief Maximum input force [N] or angular velocity [rad/s] for the
  /// propellerController, default: 1000
  public: double cmdMax = 1000;

  /// \brief Minimum input force [N] or angular velocity [rad/s] for the
  /// propellerController, default: -1000
  public: double cmdMin = -1000;

  /// \brief Thrust coefficient relating the propeller angular velocity to the
  /// thrust
  public: double thrustCoefficient = 1;

  /// \brief True if the thrust coefficient was set by configuration.
  public: bool thrustCoefficientSet = false;

  /// \brief Relative speed reduction between the water at the propeller vs
  /// behind the vessel.
  public: double wakeFraction = 0.2;

  /// \brief Constant given by the open water propeller diagram. Used in the
  /// calculation of the thrust coefficient.
  public: double alpha1 = 1;

  /// \brief Constant given by the open water propeller diagram. Used in the
  /// calculation of the thrust coefficient.
  public: double alpha2 = 0;

  /// \brief Density of fluid in kgm^-3, default: 1000kgm^-3
  public: double fluidDensity = 1000;

  /// \brief Diameter of propeller in m, default: 0.02
  public: double propellerDiameter = 0.02;

  /// \brief Thruster input voltage
  public: u_char voltage = 12;

  /// \brief An array to map ESC input (0-255 instead of 1100ms to 1900ms) to thrust in N
  public: std::array<double, 256> ESCInputToThrust = {};

  /// \brief An array to map ESC input (0-255 instead of 1100ms to 1900ms) to angular velocity in rad/s
  public: std::array<double, 256> ESCInputToAngVel = {};

  /// \brief Linear velocity of the vehicle.
  public: double linearVelocity = 0.0;

  /// \brief deadband in newtons
  public: double deadband = 0.0;

  /// \brief Flag to enable/disable deadband
  public: bool enableDeadband = false;

  /// \brief Mutex to protect enableDeadband
  public: std::mutex deadbandMutex;

  /// \brief Topic name used to enable/disable the deadband
  public: std::string deadbandTopic = "";

  /// \brief Topic name used to control thrust. Optional
  public: std::string topic = "";

  /// \brief Battery entity used by the thruster to consume power.
  public: std::string batteryName = "";

  /// \brief Battery power load of the thruster.
  public: double powerLoad = 0.0;

  /// \brief Has the battery consumption being initialized.
  public: bool batteryInitialized = false;

  /// \brief Callback for handling thrust update
  public: void OnCmdThrust(const msgs::Double &_msg);

  /// \brief Callback for handling deadband enable/disable update
  /// \param[in] _msg boolean msg to indicate whether to enable or disable
  ///                 the deadband
  public: void OnDeadbandEnable(const msgs::Boolean &_msg);

  /// \brief Recalculates and updates the thrust coefficient.
  public: void UpdateThrustCoefficient();

  /// \brief callback for handling angular velocity update
  public: void OnCmdAngVel(const gz::msgs::Double &_msg);

  /// \brief callback for handling ESC command update
  public: void OnCmdESC(const gz::msgs::Int32 &_msg);

  /// \brief Function which computes angular velocity from thrust
  /// \param[in] _thrust Thrust in N
  /// \return Angular velocity in rad/s
  public: double ThrustToAngularVec(double _thrust);

  /// \brief Function which computers thrust from angular velocity
  /// \param[in] _angVel Angular Velocity in rad/s
  /// \return Thrust in Newtons
  public: double AngularVelToThrust(double _angVel);

  /// \brief Returns a boolean if the battery has sufficient charge to continue
  /// \return True if battery is charged, false otherwise. If no battery found,
  /// returns true.
  public: bool HasSufficientBattery(const EntityComponentManager &_ecm) const;

  /// \brief Applies the deadband to the thrust and angular velocity by setting
  /// those values to zero if the thrust absolute value is below the deadband
  /// \param[in] _thrust thrust in N used for check
  /// \param[in] _angvel angular velocity in rad/s
  public: void ApplyDeadband(double &_thrust, double &_angVel);
};

/////////////////////////////////////////////////
Thruster::Thruster():
  dataPtr(std::make_unique<ThrusterPrivateData>())
{
  // do nothing
}

/////////////////////////////////////////////////
void Thruster::Configure(
  const Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  EntityComponentManager &_ecm,
  EventManager &/*_eventMgr*/)
{
  // Create model object, to access convenient functions
  this->dataPtr->modelEntity = _entity;
  auto model = Model(_entity);
  auto modelName = model.Name(_ecm);

  // Get namespace
  std::string ns = modelName;
  if (_sdf->HasElement("namespace"))
  {
    ns = _sdf->Get<std::string>("namespace");
  }

  // Get joint name
  if (!_sdf->HasElement("joint_name"))
  {
    gzerr << "Missing <joint_name>. Plugin won't be initialized."
           << std::endl;
    return;
  }
  auto jointName = _sdf->Get<std::string>("joint_name");

  // Get thrust coefficient
  if (_sdf->HasElement("thrust_coefficient"))
  {
    this->dataPtr->thrustCoefficient = _sdf->Get<double>("thrust_coefficient");
    this->dataPtr->thrustCoefficientSet = true;
  }

  // Get propeller diameter
  if (_sdf->HasElement("propeller_diameter"))
  {
    this->dataPtr->propellerDiameter = _sdf->Get<double>("propeller_diameter");
  }

  // Get fluid density, default to water otherwise
  if (_sdf->HasElement("fluid_density"))
  {
    this->dataPtr->fluidDensity = _sdf->Get<double>("fluid_density");
  }

  // Get the operation mode
  if (_sdf->HasElement("use_esc_cmd"))
  {
    this->dataPtr->opmode = _sdf->Get<bool>("use_esc_cmd") ?
      ThrusterPrivateData::OperationMode::ESCCmd :
      ThrusterPrivateData::OperationMode::ForceCmd;
  }
  else if (_sdf->HasElement("use_force_cmd"))
  {
    this->dataPtr->opmode = _sdf->Get<bool>("use_force_cmd") ?
      ThrusterPrivateData::OperationMode::ForceCmd :
      ThrusterPrivateData::OperationMode::ESCCmd;
  }
  else if (_sdf->HasElement("use_angvel_cmd"))
  {
    this->dataPtr->opmode = _sdf->Get<bool>("use_angvel_cmd") ?
      ThrusterPrivateData::OperationMode::AngVelCmd :
      ThrusterPrivateData::OperationMode::ESCCmd;
  }

  if (this->dataPtr->opmode == ThrusterPrivateData::OperationMode::ESCCmd)
  {
    // Get thruster input voltage, default to 12V otherwise
    if (_sdf->HasElement("thruster_input_voltage"))
    {
      // Get the input voltage
      int inputVoltage = _sdf->Get<int>("voltage");

      // Define the range of possible voltages
      std::vector<u_char> possibleVoltages = {10, 12, 14, 16, 18, 20};

      // Select the closest voltage to the input voltage
      this->dataPtr->voltage = *std::min_element(possibleVoltages.begin(), possibleVoltages.end(),
                  [inputVoltage](int a, int b) {
                  return std::abs(a - inputVoltage) < std::abs(b - inputVoltage);
                  });

      gzmsg << "Thruster input voltage set to " << this->dataPtr->voltage
            << "V" << std::endl;
    }

    std::string plugin_path = getenv("GZ_SIM_SYSTEM_PLUGIN_PATH");
    if (plugin_path.empty())
    {
      // If the plugin is running out the the EER Simulation environment, this condition
      // shoudn't happend.
      gzerr << "GZ_SIM_SYSTEM_PLUGIN_PATH not set. Plugin won't be initialized."
            << std::endl;
      return;
    } else 
    {
      // Determine the file path based on the voltage
      std::string file_path = plugin_path + "/plugins/T200_Thruster_Behavior/" + 
              std::vector<std::string>{
                "T200_Thruster_Behavior_10V.csv",
                "T200_Thruster_Behavior_12V.csv",
                "T200_Thruster_Behavior_14V.csv",
                "T200_Thruster_Behavior_16V.csv",
                "T200_Thruster_Behavior_18V.csv",
                "T200_Thruster_Behavior_20V.csv"
              }[((this->dataPtr->voltage) - 10) / 2];

        std::ifstream file(file_path);
        if (!file.is_open())
        {
          gzerr << "Plugin will not run. Failed to open file: " << file_path << std::endl;
          return;
        }

        short int current_index = 0;
        double previous_angular_velocity = 0;
        double previous_thrust = 0;

        std::string current_line;

        // Read the first line (CSV header). This will be disregarded
        std::getline(file, current_line);
        
        while (std::getline(file, current_line))
        {
          // Create a string stream object to parse the line for values and a temporary entry to store the values
          std::istringstream string_stream(current_line);
          std::string value;

          // Read the first value (uint8 input) as uint8. Use a static cast on the stoi function (string to integer)
          std::getline(string_stream, value, ',');
          short int file_row_index = static_cast<short int>(std::stoi(value));

          // Read the second value (RPM) as an double and convert it to radians per second (1 RPM = 0.1047rad/s). Use the stod function (string to double)
          std::getline(string_stream, value, ',');
          double current_angular_velocity = std::stod(value) * 0.1047;

          // Read the third value (Force) as a double
          std::getline(string_stream, value, ',');
          double current_thrust = std::stod(value);

          // Ignore the fourth value (pwm)
          std::getline(string_stream, value, ',');
          
          // The data provided by BlueRobotics contains 202 data points, while the ESC input has 256 possible values.
          // To fill the ESCInputToThrust and ESCInputToAngVel arrays, missing data points will be linearlly interpolated.
          while (current_index < file_row_index)
          {
            // Calculate the difference between the current and next index and add 1 (to linearlly interpolate the current index)
            short int difference = file_row_index - current_index + 1;

            this->dataPtr->ESCInputToThrust[current_index] = previous_thrust + (current_thrust - previous_thrust)/difference;
            this->dataPtr->ESCInputToAngVel[current_index] = previous_angular_velocity + (current_angular_velocity - previous_angular_velocity)/difference;

            // Update the current values
            previous_thrust = this->dataPtr->ESCInputToThrust[current_index];
            previous_angular_velocity = this->dataPtr->ESCInputToAngVel[current_index];
            current_index++;
          }

          // Fill the ESCInputToThrust and ESCInputToAngVel arrays with the obtained values
          this->dataPtr->ESCInputToThrust[file_row_index] = current_thrust;
          this->dataPtr->ESCInputToAngVel[file_row_index] = current_angular_velocity;
          
          // Update the current values
          previous_thrust = current_thrust;
          previous_angular_velocity = current_angular_velocity;
          current_index++;

        }
        file.close();
    }
  }

  // Get wake fraction number, default 0.2 otherwise
  if (_sdf->HasElement("wake_fraction"))
  {
    this->dataPtr->wakeFraction = _sdf->Get<double>("wake_fraction");
  }

  // Get alpha_1, default to 1 othwewise
  if (_sdf->HasElement("alpha_1"))
  {
    this->dataPtr->alpha1 = _sdf->Get<double>("alpha_1");
    if (this->dataPtr->thrustCoefficientSet)
    {
      gzwarn << " The [alpha_2] value will be ignored as a "
              << "[thrust_coefficient] was also defined through the SDF file."
              << " If you want the system to use the alpha values to calculate"
              << " and update the thrust coefficient please remove the "
              << "[thrust_coefficient] value from the SDF file." << std::endl;
    }
  }

  // Get alpha_2, default to 1 othwewise
  if (_sdf->HasElement("alpha_2"))
  {
    this->dataPtr->alpha2 = _sdf->Get<double>("alpha_2");
    if (this->dataPtr->thrustCoefficientSet)
    {
      gzwarn << " The [alpha_2] value will be ignored as a "
              << "[thrust_coefficient] was also defined through the SDF file."
              << " If you want the system to use the alpha values to calculate"
              << " and update the thrust coefficient please remove the "
              << "[thrust_coefficient] value from the SDF file." << std::endl;
    }
  }

  // Get deadband, default to 0
  // Note that the deadband will not manually applied regardless in ESCCmd mode (deadband is already included in thruster data)
  if (_sdf->HasElement("deadband") && this->dataPtr->opmode != ThrusterPrivateData::OperationMode::ESCCmd)
  {
    this->dataPtr->deadband = _sdf->Get<double>("deadband");
    this->dataPtr->enableDeadband = true;
  }

  // Get a custom topic.
  if (_sdf->HasElement("topic"))
  {
    this->dataPtr->topic = transport::TopicUtils::AsValidTopic(
      _sdf->Get<std::string>("topic"));
  }

  this->dataPtr->jointEntity = model.JointByName(_ecm, jointName);
  if (kNullEntity == this->dataPtr->jointEntity)
  {
    gzerr << "Failed to find joint [" << jointName << "] in model ["
           << modelName << "]. Plugin not initialized." << std::endl;
    return;
  }

  this->dataPtr->jointAxis =
    _ecm.Component<components::JointAxis>(
    this->dataPtr->jointEntity)->Data().Xyz();

  this->dataPtr->jointPose = _ecm.Component<components::Pose>(
      this->dataPtr->jointEntity)->Data();

  // Get link entity
  auto childLink =
      _ecm.Component<components::ChildLinkName>(
      this->dataPtr->jointEntity);

  this->dataPtr->linkEntity = model.LinkByName(_ecm, childLink->Data());

  std::string thrusterTopic;
  std::string feedbackTopic;
  std::string feedbackTopic2;
  if (!this->dataPtr->topic.empty())
  {
    // Subscribe to specified topic for force commands
    thrusterTopic = gz::transport::TopicUtils::AsValidTopic(
      ns + "/" + this->dataPtr->topic);
    this->dataPtr->deadbandTopic = gz::transport::TopicUtils::AsValidTopic(
      ns + "/" + this->dataPtr->topic + "/enable_deadband");
    if (this->dataPtr->opmode == ThrusterPrivateData::OperationMode::ESCCmd)
    {
      this->dataPtr->node.Subscribe(
          thrusterTopic,
          &ThrusterPrivateData::OnCmdESC,
          this->dataPtr.get());

      feedbackTopic = gz::transport::TopicUtils::AsValidTopic(
        ns + "/" + this->dataPtr->topic + "/force");

      feedbackTopic2 = gz::transport::TopicUtils::AsValidTopic(
        ns + "/" + this->dataPtr->topic + + "/ang_vel");
    }
    else if (this->dataPtr->opmode == ThrusterPrivateData::OperationMode::ForceCmd)
    {
      this->dataPtr->node.Subscribe(
          thrusterTopic,
          &ThrusterPrivateData::OnCmdThrust,
          this->dataPtr.get());

      feedbackTopic = gz::transport::TopicUtils::AsValidTopic(
        ns + "/" + this->dataPtr->topic + "/ang_vel");
    }
    else
    {
      this->dataPtr->node.Subscribe(
        thrusterTopic,
        &ThrusterPrivateData::OnCmdAngVel,
        this->dataPtr.get());

      feedbackTopic = gz::transport::TopicUtils::AsValidTopic(
          ns + "/" + this->dataPtr->topic + "/force");
    }
  }
  else if (this->dataPtr->opmode ==
           ThrusterPrivateData::OperationMode::ESCCmd)
  {
    // Subscribe to ESC commands
    thrusterTopic = gz::transport::TopicUtils::AsValidTopic(
      "/model/" + ns + "/joint/" + jointName + "/cmd_esc");

    this->dataPtr->node.Subscribe(
      thrusterTopic,
      &ThrusterPrivateData::OnCmdESC,
      this->dataPtr.get());

    feedbackTopic = gz::transport::TopicUtils::AsValidTopic(
      "/model/" + ns + "/joint/" + jointName + "/force");

    feedbackTopic2 = gz::transport::TopicUtils::AsValidTopic(
      "/model/" + ns + "/joint/" + jointName + "/ang_vel");
  }
  else if (this->dataPtr->opmode ==
           ThrusterPrivateData::OperationMode::ForceCmd)
  {
    // Subscribe to force commands
    thrusterTopic = gz::transport::TopicUtils::AsValidTopic(
      "/model/" + ns + "/joint/" + jointName + "/cmd_thrust");

    this->dataPtr->node.Subscribe(
      thrusterTopic,
      &ThrusterPrivateData::OnCmdThrust,
      this->dataPtr.get());

    feedbackTopic = gz::transport::TopicUtils::AsValidTopic(
      "/model/" + ns + "/joint/" + jointName + "/ang_vel");

    this->dataPtr->deadbandTopic = gz::transport::TopicUtils::AsValidTopic(
      "/model/" + ns + "/joint/" + jointName + "/enable_deadband");
  }
  else
  {
    gzdbg << "Using angular velocity mode" << std::endl;
    // Subscribe to angvel commands
    thrusterTopic = gz::transport::TopicUtils::AsValidTopic(
      "/model/" + ns + "/joint/" + jointName + "/cmd_vel");

    this->dataPtr->node.Subscribe(
      thrusterTopic,
      &ThrusterPrivateData::OnCmdAngVel,
      this->dataPtr.get());

    feedbackTopic = gz::transport::TopicUtils::AsValidTopic(
        "/model/" + ns + "/joint/" + jointName + "/force");

    this->dataPtr->deadbandTopic = gz::transport::TopicUtils::AsValidTopic(
      "/model/" + ns + "/joint/" + jointName + "/enable_deadband");
  }

  gzmsg << "Thruster listening to commands on [" << thrusterTopic << "]"
        << std::endl;

  if (!this->dataPtr->deadbandTopic.empty())
  {
    this->dataPtr->node.Subscribe(
        this->dataPtr->deadbandTopic,
        &ThrusterPrivateData::OnDeadbandEnable,
        this->dataPtr.get());
    gzmsg << "Thruster listening to enable_deadband on ["
          << this->dataPtr->deadbandTopic << "]" << std::endl;
  }

  this->dataPtr->pub = this->dataPtr->node.Advertise<msgs::Double>(
      feedbackTopic);

  if (!feedbackTopic2.empty()) {
    this->dataPtr->pub2 = this->dataPtr->node.Advertise<msgs::Double>(
        feedbackTopic2);
  }

  // Create necessary components if not present.
  enableComponent<components::AngularVelocity>(_ecm, this->dataPtr->linkEntity);
  enableComponent<components::WorldAngularVelocity>(_ecm,
      this->dataPtr->linkEntity);
  enableComponent<components::WorldLinearVelocity>(_ecm,
      this->dataPtr->linkEntity);

  double minThrustCmd = this->dataPtr->cmdMin;
  double maxThrustCmd = this->dataPtr->cmdMax;
  if (_sdf->HasElement("max_thrust_cmd"))
  {
    maxThrustCmd = _sdf->Get<double>("max_thrust_cmd");
  }
  if (_sdf->HasElement("min_thrust_cmd"))
  {
    minThrustCmd = _sdf->Get<double>("min_thrust_cmd");
  }
  if (maxThrustCmd < minThrustCmd)
  {
    gzerr << "<max_thrust_cmd> must be greater than or equal to "
           << "<min_thrust_cmd>. Revert to using default values: "
           << "min: " << this->dataPtr->cmdMin << ", "
           << "max: " << this->dataPtr->cmdMax << std::endl;
  }
  else
  {
    this->dataPtr->cmdMax = maxThrustCmd;
    this->dataPtr->cmdMin = minThrustCmd;
  }

  if (_sdf->HasElement("velocity_control"))
  {
    this->dataPtr->velocityControl = _sdf->Get<bool>("velocity_control");
  }

  if (!this->dataPtr->velocityControl)
  {
    gzdbg << "Using PID controller for propeller joint." << std::endl;

    double p         =  0.1;
    double i         =  0;
    double d         =  0;
    double iMax      =  1;
    double iMin      = -1;
    double cmdMax    = this->dataPtr->ThrustToAngularVec(this->dataPtr->cmdMax);
    double cmdMin    = this->dataPtr->ThrustToAngularVec(this->dataPtr->cmdMin);
    double cmdOffset =  0;

    if (_sdf->HasElement("p_gain"))
    {
      p = _sdf->Get<double>("p_gain");
    }
    if (!_sdf->HasElement("i_gain"))
    {
      i = _sdf->Get<double>("i_gain");
    }
    if (!_sdf->HasElement("d_gain"))
    {
      d = _sdf->Get<double>("d_gain");
    }

    this->dataPtr->propellerController.Init(
      p,
      i,
      d,
      iMax,
      iMin,
      cmdMax,
      cmdMin,
      cmdOffset);
  }
  else
  {
    gzdbg << "Using velocity control for propeller joint." << std::endl;
  }

  // Get power load and battery name info
  if (_sdf->HasElement("power_load"))
  {
    if (!_sdf->HasElement("battery_name"))
    {
      gzerr << "Specified a <power_load> but missing <battery_name>."
          "Specify a battery name so the power load can be assigned to it."
          << std::endl;
    }
    else
    {
      this->dataPtr->powerLoad = _sdf->Get<double>("power_load");
      this->dataPtr->batteryName = _sdf->Get<std::string>("battery_name");
    }
  }
}

/////////////////////////////////////////////////
void ThrusterPrivateData::OnCmdThrust(const gz::msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(mtx);
  this->thrust = gz::math::clamp(gz::math::fixnan(_msg.data()),
    this->cmdMin, this->cmdMax);

  // Thrust is proportional to the Rotation Rate squared
  // See Thor I Fossen's  "Guidance and Control of ocean vehicles" p. 246
  this->propellerAngVel = this->ThrustToAngularVec(this->thrust);
}

/////////////////////////////////////////////////
void ThrusterPrivateData::OnDeadbandEnable(const gz::msgs::Boolean &_msg)
{
  std::lock_guard<std::mutex> lock(this->deadbandMutex);
  if (_msg.data() != this->enableDeadband)
  {
    if (_msg.data())
    {
      gzmsg << "Enabling deadband." << std::endl;
    }
    else
    {
      gzmsg << "Disabling deadband." << std::endl;
    }

    this->enableDeadband = _msg.data();
  }

}

/////////////////////////////////////////////////
void ThrusterPrivateData::OnCmdAngVel(const gz::msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(mtx);
  this->propellerAngVel =
    gz::math::clamp(gz::math::fixnan(_msg.data()),
      this->cmdMin, this->cmdMax);

  // Thrust is proportional to the Rotation Rate squared
  // See Thor I Fossen's  "Guidance and Control of ocean vehicles" p. 246
  this->thrust = this->AngularVelToThrust(this->propellerAngVel);
}

/////////////////////////////////////////////////
double ThrusterPrivateData::ThrustToAngularVec(double _thrust)
{
  // Only update if the thrust coefficient was not set by configuration
  // and angular velocity is not zero. Some velocity is needed to calculate
  // the thrust coefficient otherwise it will never start moving.
  if (!this->thrustCoefficientSet &&
      std::abs(this->propellerAngVel) > std::numeric_limits<double>::epsilon())
  {
    this->UpdateThrustCoefficient();
  }
  // Thrust is proportional to the Rotation Rate squared
  // See Thor I Fossen's  "Guidance and Control of ocean vehicles" p. 246
  auto propAngularVelocity = sqrt(abs(
    _thrust /
      (this->fluidDensity
      * this->thrustCoefficient * pow(this->propellerDiameter, 4))));

  propAngularVelocity *= (_thrust * this->thrustCoefficient > 0) ? 1: -1;

  return propAngularVelocity;
}

/////////////////////////////////////////////////
void ThrusterPrivateData::OnCmdESC(const gz::msgs::Int32 &_msg)
{
  std::lock_guard<std::mutex> lock(mtx);
  // Simply obtain the thrust and angular velocity from the ESCInputToThrust and ESCInputToAngVel arrays
  this->thrust = this->ESCInputToThrust[static_cast<uint8_t>(_msg.data())];
  this->propellerAngVel = this->ESCInputToAngVel[static_cast<uint8_t>(_msg.data())];
}

/////////////////////////////////////////////////
void ThrusterPrivateData::UpdateThrustCoefficient()
{
  this->thrustCoefficient = this->alpha1 + this->alpha2 *
      (((1 - this->wakeFraction) * this->linearVelocity)
      / (this->propellerAngVel * this->propellerDiameter));
}

/////////////////////////////////////////////////
double ThrusterPrivateData::AngularVelToThrust(double _angVel)
{
  // Thrust is proportional to the Rotation Rate squared
  // See Thor I Fossen's  "Guidance and Control of ocean vehicles" p. 246
  return this->thrustCoefficient * pow(this->propellerDiameter, 4)
    * abs(_angVel) * _angVel * this->fluidDensity;
}

/////////////////////////////////////////////////
bool ThrusterPrivateData::HasSufficientBattery(
  const EntityComponentManager &_ecm) const
{
  bool result = true;
  _ecm.Each<components::BatterySoC>([&](
    const Entity &_entity,
    const components::BatterySoC *_data
  ){
    if(_ecm.ParentEntity(_entity) == this->modelEntity)
    {
      if(_data->Data() <= 0)
      {
        result = false;
      }
    }

    return true;
  });
  return result;
}

/////////////////////////////////////////////////
void ThrusterPrivateData::ApplyDeadband(double &_thrust, double &_angVel)
{
    if (abs(_thrust) < this->deadband)
    {
        _thrust = 0.0;
        _angVel = 0.0;
    }
}

/////////////////////////////////////////////////
void Thruster::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  if (!this->dataPtr->enabled)
  {
    return;
  }
  if (!_ecm.HasEntity(this->dataPtr->linkEntity)){
    return;
  }

  // Init battery consumption if it was set
  if (!this->dataPtr->batteryName.empty() &&
      !this->dataPtr->batteryInitialized)
  {
    this->dataPtr->batteryInitialized = true;

    // Check that a battery exists with the specified name
    Entity batteryEntity;
    int numBatteriesWithName = 0;
    _ecm.Each<components::BatterySoC, components::Name>(
      [&](const Entity &_entity,
        const components::BatterySoC */*_BatterySoC*/,
        const components::Name *_name)->bool
      {
        if (this->dataPtr->batteryName == _name->Data())
        {
          ++numBatteriesWithName;
          batteryEntity = _entity;
        }
        return true;
      });
    if (numBatteriesWithName == 0)
    {
      gzerr << "Can't assign battery consumption to battery: ["
            << this->dataPtr->batteryName << "]. No batteries"
            "were found with the given name." << std::endl;
      return;
    }
    if (numBatteriesWithName > 1)
    {
      gzerr << "More than one battery found with name: ["
            << this->dataPtr->batteryName << "]. Please make"
            "sure battery names are unique within the system."
            << std::endl;
      return;
    }

    // Create the battery consumer entity and its component
    this->dataPtr->consumerEntity = _ecm.CreateEntity();
    components::BatteryPowerLoadInfo batteryPowerLoadInfo{
        batteryEntity, this->dataPtr->powerLoad};
    _ecm.CreateComponent(this->dataPtr->consumerEntity,
        components::BatteryPowerLoad(batteryPowerLoadInfo));
    _ecm.SetParentEntity(this->dataPtr->consumerEntity, batteryEntity);
  }

  gz::sim::Link link(this->dataPtr->linkEntity);

  // TODO(arjo129): add logic for custom coordinate frame
  // Convert joint axis to the world frame
  const auto linkWorldPose = worldPose(this->dataPtr->linkEntity, _ecm);
  auto jointWorldPose = linkWorldPose * this->dataPtr->jointPose;
  auto unitVector =
      jointWorldPose.Rot().RotateVector(this->dataPtr->jointAxis).Normalize();

  double desiredThrust;
  double desiredPropellerAngVel;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mtx);
    desiredThrust = this->dataPtr->thrust;
    if (!this->dataPtr->opmode == ThrusterPrivateData::OperationMode::ESCCmd)
    {
      this->dataPtr->propellerAngVel = this->dataPtr->ThrustToAngularVec(this->dataPtr->thrust);
    }
    // The angular velocity is already set in ESCCmd mode
    desiredPropellerAngVel = this->dataPtr->propellerAngVel;
  }

  {
    std::lock_guard<std::mutex> lock(this->dataPtr->deadbandMutex);
    if (this->dataPtr->enableDeadband)
    {
      this->dataPtr->ApplyDeadband(desiredThrust, desiredPropellerAngVel);
    }
  }


  msgs::Double angvel;
  // PID control
  double torque = 0.0;
  if (!this->dataPtr->velocityControl)
  {
    auto currentAngular = (link.WorldAngularVelocity(_ecm))->Dot(unitVector);
    auto angularError = currentAngular - desiredPropellerAngVel;
    if (abs(angularError) > 0.1)
    {
      torque = this->dataPtr->propellerController.Update(angularError,
          _info.dt);
    }
    angvel.set_data(currentAngular);
  }
  // Velocity control
  else
  {
    _ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(
      this->dataPtr->jointEntity, {desiredPropellerAngVel});
    angvel.set_data(desiredPropellerAngVel);
  }

  if (this->dataPtr->opmode == ThrusterPrivateData::OperationMode::ESCCmd)
  {
    msgs::Double force;
    force.set_data(desiredThrust);
    this->dataPtr->pub.Publish(force);
    this->dataPtr->pub2.Publish(angvel);
  }
  else if (this->dataPtr->opmode == ThrusterPrivateData::OperationMode::ForceCmd)
  {
    this->dataPtr->pub.Publish(angvel);
  }
  else
  {
    msgs::Double force;
    force.set_data(desiredThrust);
    this->dataPtr->pub.Publish(force);
  }
  // Force: thrust
  // Torque: propeller rotation, if using PID
  link.AddWorldWrench(
    _ecm,
    unitVector * desiredThrust,
    unitVector * torque);

  // Update the LinearVelocity of the vehicle
  this->dataPtr->linearVelocity =
      _ecm.Component<components::WorldLinearVelocity>(
      this->dataPtr->linkEntity)->Data().Length();
}

/////////////////////////////////////////////////
void Thruster::PostUpdate(const UpdateInfo &/*unused*/,
  const EntityComponentManager &_ecm)
{
  this->dataPtr->enabled = this->dataPtr->HasSufficientBattery(_ecm);
}

GZ_ADD_PLUGIN(
  Thruster, System,
  Thruster::ISystemConfigure,
  Thruster::ISystemPreUpdate,
  Thruster::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(Thruster, "gz::sim::systems::EER_Thruster")