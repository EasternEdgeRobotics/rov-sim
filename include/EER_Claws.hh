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
#ifndef GZ_SIM_SYSTEMS_Claws_HH_
#define GZ_SIM_SYSTEMS_Claws_HH_

#include <gz/sim/System.hh>
#include <memory>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class ClawsPrivate;

  /// \brief Joint controller which can be attached to a model with a reference
  /// to a single joint. Currently only the first axis of a joint is actuated.
  ///
  /// ## System Parameters
  ///
  /// - `<right_claw>` A tag under which the right claw parameters are defined (required).
  ///
  /// - `<left_claw>` A tag under which the left claw parameters are defined (required).
  ///
  /// - `<joint>` The name of the claw joint to control (required for each claw).  
  ///
  /// - `<max_inward_rotation>` The maximum inward rotation of the claw in radians relative to the starting position (required for each claw). 
  ///
  /// - `<max_outward_rotation>` The maximum outward rotation of the claw in radians relative to the starting position (required for each claw).
  ///
  /// - `<topic>` Topic to receive commands in. Defaults to
  ///     `/<model_name>/claws/cmd_vel`.
  ///
  /// - `<sub_topic>` Sub topic to receive commands in.
  /// Defaults to "/model/<model_name>/<sub_topic>".
  
  class Claws
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: Claws();

    /// \brief Destructor
    public: ~Claws() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<ClawsPrivate> dataPtr;
  };
  }
}
}
}

#endif