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
#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPixmap>
#include <QProgressBar>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include <chrono>
#include <cstdio>
#include <iomanip>

#include <tf2/utils.h>

#include "rviz_common/display_context.hpp"
#include <OgreCamera.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <rviz_common/view_manager.hpp>

#include "control_panel.h"
#include "indicator_widget.h"

using std::placeholders::_1;
namespace scenario_execution_rviz {

ControlPanel::ControlPanel(QWidget *parent) : rviz_common::Panel(parent) {
  QVBoxLayout *layout = new QVBoxLayout;

  QFormLayout *formLayout = new QFormLayout;

  QHBoxLayout *rowLayout = new QHBoxLayout;

  mScenarioSelection = new QComboBox();
  rowLayout->addWidget(mScenarioSelection, 10);

  QPixmap pixmap(":/icons/play.png");
  QIcon iconPlay(pixmap);
  mTriggerScenarioButton = new QPushButton(iconPlay, "");
  rowLayout->addWidget(mTriggerScenarioButton);

  mIndicatorWidget = new IndicatorWidget();
  rowLayout->addWidget(mIndicatorWidget);
  connect(mTriggerScenarioButton, SIGNAL(released()), this,
          SLOT(scenarioExecuteButtonClicked()));

  formLayout->addRow(rowLayout);

  layout->addLayout(formLayout);

  setLayout(layout);

  setScenarioExecutionStatus(false);
}

void ControlPanel::onInitialize() {
  _node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  mExecuteScenarioClient =
      _node->create_client<scenario_execution_interfaces::srv::ExecuteScenario>(
          "/scenario_execution_control/execute_scenario");
  mStopScenarioClient = _node->create_client<std_srvs::srv::Empty>(
      "/scenario_execution_control/stop_scenario");
  mScenarioExecutionStatusSubscriber = _node->create_subscription<
      scenario_execution_interfaces::msg::ScenarioExecutionStatus>(
      "/scenario_execution_control/status", 10,
      std::bind(&ControlPanel::scenarioExecutionStatusChanged, this, _1));
  rclcpp::QoS static_QoS = rclcpp::QoS(1).transient_local();
  mScenarioSubscriber = _node->create_subscription<
      scenario_execution_interfaces::msg::ScenarioList>(
      "/scenario_execution_control/available_scenarios", static_QoS,
      std::bind(&ControlPanel::scenariosChanged, this, _1));
}

void ControlPanel::scenarioExecuteButtonClicked() {
  if (!mScenarioIsRunning) {
    for (auto const &scenario : mScenarios->scenarios) {
      if (QString::fromStdString(scenario.name) ==
          mScenarioSelection->currentText()) {
        auto request = std::make_shared<
            scenario_execution_interfaces::srv::ExecuteScenario::Request>();
        request->scenario = scenario;
        // Check if service is available
        if (!mExecuteScenarioClient->wait_for_service(
                std::chrono::seconds(1))) {
          RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                      "Failed to call service executeScenario");
          mIndicatorWidget->setState(IndicatorWidget::State::Error);
        }
        auto result = mExecuteScenarioClient->async_send_request(request);
        break;
      }
    }
  } else {
    if (!mStopScenarioClient->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                  "Failed to call service stopScenario");
      mIndicatorWidget->setState(IndicatorWidget::State::Error);
    }
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto result = mStopScenarioClient->async_send_request(request);
  }
}

void ControlPanel::scenarioExecutionStatusChanged(
    const scenario_execution_interfaces::msg::ScenarioExecutionStatus::SharedPtr
        msg) {
  bool isRunning = false;
  if (msg->status ==
      scenario_execution_interfaces::msg::ScenarioExecutionStatus::STOPPED) {
    mIndicatorWidget->setState(IndicatorWidget::State::Stopped);
  } else if (msg->status == scenario_execution_interfaces::msg::
                                ScenarioExecutionStatus::STARTING) {
    mIndicatorWidget->setState(IndicatorWidget::State::Starting);
    isRunning = true;
  } else if (msg->status == scenario_execution_interfaces::msg::
                                ScenarioExecutionStatus::RUNNING) {
    mIndicatorWidget->setState(IndicatorWidget::State::Running);
    isRunning = true;
  } else if (msg->status == scenario_execution_interfaces::msg::
                                ScenarioExecutionStatus::SHUTTINGDOWN) {
    mIndicatorWidget->setState(IndicatorWidget::State::ShuttingDown);
    isRunning = true;
  } else {
    mIndicatorWidget->setState(IndicatorWidget::State::Error);
  }

  updateScenarioExecutionRunning(isRunning);
}

void ControlPanel::updateScenarioExecutionRunning(bool isRunning) {
  mScenarioIsRunning = isRunning;
  if (isRunning) {
    QPixmap pixmapStop(":/icons/stop.png");
    QIcon iconStop(pixmapStop);
    mTriggerScenarioButton->setIcon(iconStop);
  } else {
    QPixmap pixmapPlay(":/icons/play.png");
    QIcon iconPlay(pixmapPlay);
    mTriggerScenarioButton->setIcon(iconPlay);
  }
}

void ControlPanel::setScenarioExecutionStatus(bool active) {
  mScenarioSelection->setEnabled(active);
  mTriggerScenarioButton->setEnabled(active);
  mIndicatorWidget->setEnabled(active);
}

void ControlPanel::scenariosChanged(
    const scenario_execution_interfaces::msg::ScenarioList::SharedPtr msg) {
  auto currentSelection = mScenarioSelection->currentText();
  mScenarios = msg;
  mScenarioSelection->clear();
  int idx = 0;
  QStringList tmpScenarios;
  for (auto const &scenario : msg->scenarios) {
    auto name = QString::fromStdString(scenario.name);
    tmpScenarios.append(name);
  }
  tmpScenarios.sort(Qt::CaseInsensitive);
  for (auto const &scenario : tmpScenarios) {
    mScenarioSelection->addItem(scenario);
    if (scenario == currentSelection) { // switch to previously selected item
      mScenarioSelection->setCurrentIndex(idx);
    }
    ++idx;
  }
  setScenarioExecutionStatus(mScenarioSelection->count() > 0);
}

} // end namespace scenario_execution_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(scenario_execution_rviz::ControlPanel,
                       rviz_common::Panel)
