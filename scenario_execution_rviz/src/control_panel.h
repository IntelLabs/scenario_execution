//  Copyright (C) 2024 Intel Corporation
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing,
//  software distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions
//  and limitations under the License.
//
//  SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (c) 2020 Intel Corporation
 *
 * This work is licensed under the terms of the MIT license.
 * For a copy, see <https://opensource.org/licenses/MIT>.
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include <rviz_common/panel.hpp>
#include <scenario_execution_interfaces/msg/scenario_execution_status.hpp>
#include <scenario_execution_interfaces/msg/scenario_list.hpp>
#include <scenario_execution_interfaces/srv/execute_scenario.hpp>
#include <std_srvs/srv/empty.hpp>

class QPushButton;
class QComboBox;

namespace scenario_execution_rviz {

class IndicatorWidget;

class ScenarioControl : public rviz_common::Panel {
  Q_OBJECT
public:
  ScenarioControl(QWidget *parent = 0);

protected Q_SLOTS:
  void scenarioExecuteButtonClicked();

protected:
  virtual void onInitialize() override;
  void setScenarioExecutionStatus(bool active);

  void
  scenarioExecutionStatusChanged(const scenario_execution_interfaces::msg::
                                     ScenarioExecutionStatus::SharedPtr msg);
  void scenariosChanged(
      const scenario_execution_interfaces::msg::ScenarioList::SharedPtr msg);
  void updateScenarioExecutionRunning(bool isRunning);

  rclcpp::Node::SharedPtr _node;

  QPushButton *mTriggerScenarioButton;
  QComboBox *mScenarioSelection;
  IndicatorWidget *mIndicatorWidget;
  rclcpp::Client<scenario_execution_interfaces::srv::ExecuteScenario>::SharedPtr
      mExecuteScenarioClient;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr mStopScenarioClient;
  rclcpp::Subscription<scenario_execution_interfaces::msg::ScenarioList>::
      SharedPtr mScenarioSubscriber;
  rclcpp::Subscription<
      scenario_execution_interfaces::msg::ScenarioExecutionStatus>::SharedPtr
      mScenarioExecutionStatusSubscriber;

  scenario_execution_interfaces::msg::ScenarioList::SharedPtr mScenarios;
  bool mScenarioIsRunning = false;
};

} // end namespace scenario_execution_rviz
