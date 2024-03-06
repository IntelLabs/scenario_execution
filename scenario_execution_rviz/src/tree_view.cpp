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

#include "tree_view.h"
#include "indicator_widget.h"

using std::placeholders::_1;

namespace scenario_execution_rviz {

TreeView::TreeView(QWidget *parent) : rviz_common::Panel(parent) {
  QVBoxLayout *layout = new QVBoxLayout;

  QFormLayout *formLayout = new QFormLayout;

  QHBoxLayout *rowLayout = new QHBoxLayout;
  mTreeView = new QTreeWidget();
  mTreeView->setColumnCount(3);
  mTreeView->setColumnHidden(2, true);
  mTreeView->setColumnWidth(0, 200);
  mTreeView->setColumnWidth(1, 200);

  QStringList columnNames;
  columnNames << "Action"
              << "Feedback";
  mTreeView->setHeaderLabels(columnNames);
  collapsedStates = new QMap<QString, bool>;
  rowLayout->addWidget(mTreeView);

  formLayout->addRow(rowLayout);

  layout->addLayout(formLayout);

  setLayout(layout);

  // user input
  connect(mTreeView, SIGNAL(itemCollapsed(QTreeWidgetItem *)), this,
          SLOT(handleItemCollapsed(QTreeWidgetItem *)));
  connect(mTreeView, SIGNAL(itemExpanded(QTreeWidgetItem *)), this,
          SLOT(handleItemExpanded(QTreeWidgetItem *)));

  timer = new QTimer(this);
  timer->setInterval(1000);
  connect(timer, SIGNAL(timeout()), this, SLOT(requestBtPublishing()));
  timer->start();
  qDebug() << "initialisation done";
}

void TreeView::onInitialize() {
  _node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  mBehaviorTreeSubscriber =
      _node->create_subscription<py_trees_ros_interfaces::msg::BehaviourTree>(
          "/bt_snapshot_rviz", 10,
          std::bind(&TreeView::behaviorTreeChanged, this, _1));

  mOpenSnapshotStreamClient =
      _node->create_client<py_trees_ros_interfaces::srv::OpenSnapshotStream>(
          "/scenario_execution/snapshot_streams/open");
}

void TreeView::handleItemCollapsed(QTreeWidgetItem *collapsedItem) {
  collapsedStates->insert(collapsedItem->text(2), false);
}

void TreeView::handleItemExpanded(QTreeWidgetItem *expandedItem) {
  collapsedStates->insert(expandedItem->text(2), true);
}

int searchBehavior(const QString *child_id,
                   QList<ConvertedBehavior *> *behaviorlist) {

  int childPosition = 0;
  for (auto const &behavior : *behaviorlist) {
    if (*child_id == behavior->own_id) {
      break;
    }
    childPosition = childPosition + 1;
  }
  return childPosition;
}

void TreeView::behaviorTreeChanged(
    const py_trees_ros_interfaces::msg::BehaviourTree::SharedPtr msg) {
  if (treeWidgetBuilt == false || msg->changed == true) {
    QList<QTreeWidgetItem *> items;
    mTreeView->clear();
    populateTree(items, msg);
    mTreeView->insertTopLevelItems(0, items);

    // expand everything if its run forthe first time
    // else only expand already expanded item
    if (treeWidgetBuilt) {
      for (auto const &item : items) {
        if (collapsedStates->value(item->text(2))) {
          mTreeView->expandItem(item);
        }
      }
    } else {
      for (auto const &item : items) {
        mTreeView->expandItem(item);
      }
    }

    // save the state/new state of the collapsed items in the QMap
    int qMapcounter = 0;
    collapsedStates->clear();
    for (auto const &item : items) {
      collapsedStates->insert(item->text(2), item->isExpanded());
      qMapcounter++;
    }

    treeWidgetBuilt = true;
  }
  timer->start();
}

void TreeView::populateTree(
    QList<QTreeWidgetItem *> &items,
    const py_trees_ros_interfaces::msg::BehaviourTree::SharedPtr msg) {
  QList<ConvertedBehavior *> mBehaviorList;

  // iterate from behind and create a list of behavior objects
  for (auto behavior_iter = msg->behaviours.rbegin();
       behavior_iter != msg->behaviours.rend(); ++behavior_iter) {
    auto const &behavior = *behavior_iter;
    mBehaviorList << new ConvertedBehavior(behavior);
  }

  for (auto const &behavior : mBehaviorList) {
    QStringList nodeName;
    nodeName << behavior->name << behavior->message << behavior->own_id;

    items.append(new QTreeWidgetItem(nodeName));

    if (behavior->status == 1) {
      items.last()->setIcon(0, waitingIcon);
    } else if (behavior->status == 2) {
      items.last()->setIcon(0, runningIcon);
    } else if (behavior->status == 3) {
      items.last()->setIcon(0, successIcon);
    } else if (behavior->status == 4) {
      items.last()->setIcon(0, failedIcon);
    }
  }

  int parentPosition = 0;

  for (auto const &behavior : mBehaviorList) {

    for (auto const &child_id : behavior->child_ids) {

      int childPosition = searchBehavior(&child_id, &mBehaviorList);
      items[parentPosition]->addChild(items[childPosition]);
    }

    ++parentPosition;
  }
}

void TreeView::requestBtPublishing() {
  if (!mOpenSnapshotStreamClient->wait_for_service(
          std::chrono::milliseconds(50))) {
    RCLCPP_WARN(
        rclcpp::get_logger("rclcpp"),
        "Failed to call service OpenSnapshotStream. Will try to reconnect");
    treeWidgetBuilt = false;
    timer->start();
  }
  auto request = std::make_shared<
      py_trees_ros_interfaces::srv::OpenSnapshotStream::Request>();
  request->topic_name = "/bt_snapshot_rviz";
  auto result = mOpenSnapshotStreamClient->async_send_request(request);
}

} // end namespace scenario_execution_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(scenario_execution_rviz::TreeView, rviz_common::Panel)
