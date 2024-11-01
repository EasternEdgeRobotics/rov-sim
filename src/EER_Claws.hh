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
  ///     `/model/<model_name>/joint/<joint_name>/cmd_vel`.
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