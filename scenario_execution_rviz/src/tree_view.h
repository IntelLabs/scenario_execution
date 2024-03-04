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

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include <OgreCamera.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <QCheckBox>
#include <QComboBox>
#include <QDebug>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QLineEdit>
#include <QMap>
#include <QObject>
#include <QPainter>
#include <QPixmap>
#include <QProgressBar>
#include <QPushButton>
#include <QTimer>
#include <QTreeView>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>
#include <chrono>
#include <cstdio>
#include <iomanip>
#include <py_trees_ros_interfaces/msg/behaviour_tree.hpp>
#include <py_trees_ros_interfaces/srv/open_snapshot_stream.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/view_manager.hpp>
#include <tf2/utils.h>

class QPushButton;
class QComboBox;
class QTreeView;
class TreeModel;
class QTreeWidget;

namespace scenario_execution_rviz {

class IndicatorWidget;

class TreeView : public rviz_common::Panel {
  Q_OBJECT
public:
  TreeView(QWidget *parent = 0);

protected Q_SLOTS:
  void requestBtPublishing();
  void handleItemCollapsed(QTreeWidgetItem *collapsedItem);
  void handleItemExpanded(QTreeWidgetItem *expandedItem);

protected:
  virtual void onInitialize() override;

  void behaviorTreeChanged(
      const py_trees_ros_interfaces::msg::BehaviourTree::SharedPtr msg);
  void populateTree(
      QList<QTreeWidgetItem *> &items,
      const py_trees_ros_interfaces::msg::BehaviourTree::SharedPtr msg);
  rclcpp::Subscription<py_trees_ros_interfaces::msg::BehaviourTree>::SharedPtr
      mBehaviorTreeSubscriber;
  rclcpp::Client<py_trees_ros_interfaces::srv::OpenSnapshotStream>::SharedPtr
      mOpenSnapshotStreamClient;

  rclcpp::Node::SharedPtr _node;

  QTreeWidget *mTreeView;
  TreeModel *mTreeModel;
  bool treeWidgetBuilt = false;
  QTimer *timer;
  QMap<QString, bool> *collapsedStates;

  // icons
  QIcon runningIcon = QIcon(":/icons/chevron-right-o.png");
  QIcon successIcon = QIcon(":/icons/check-o.png");
  QIcon failedIcon = QIcon(":/icons/close-o.png");
  QIcon waitingIcon = QIcon(":/icons/corner-down-right.png");
};

} // end namespace scenario_execution_rviz

class ConvertedBehavior : public QObject {

public:
  explicit ConvertedBehavior(
      const py_trees_ros_interfaces::msg::Behaviour &msg) {
    name = QString::fromStdString(msg.name);
    class_name = QString::fromStdString(msg.class_name);
    message = QString::fromStdString(msg.message);
    status = msg.status;
    type = msg.type;
    parent_id = uuidToQString(msg.parent_id.uuid);
    own_id = uuidToQString(msg.own_id.uuid);

    if (checkChild(msg.child_ids[0].uuid) == true) {

      for (auto const &child : msg.child_ids) {
        child_ids << uuidToQString(child.uuid);
      }
    }
  }

  QString name;
  QString class_name;
  QString own_id;
  QString parent_id;
  QStringList child_ids;
  QString current_child_id;
  QString message;
  int8_t status;
  int8_t type;

  QList<ConvertedBehavior *> mBehaviorList;

  QString uuidToQString(const std::array<unsigned char, 16> &uuid) {
    QString result;

    for (const auto &element : uuid) {

      result.append(QString("%1").arg(static_cast<unsigned int>(element), 2, 16,
                                      QChar('0')));
    }
    return result;
  }

  bool checkChild(const std::array<unsigned char, 16> &uuid) {
    bool result = true;
    // const unsigned char *null_ptr = NULL;
    const unsigned char *element_ptr = &uuid[0];
    if (element_ptr == nullptr) {

      result = false;
    }
    return result;
  }
};