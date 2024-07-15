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

#include "scenario_view.h"
#include "indicator_widget.h"

using std::placeholders::_1;

namespace scenario_execution_rviz {

ScenarioView::ScenarioView(QWidget *parent) : rviz_common::Panel(parent) {
  QVBoxLayout *layout = new QVBoxLayout;

  QFormLayout *formLayout = new QFormLayout;

  QHBoxLayout *rowLayout = new QHBoxLayout;
  mScenarioView = new QTreeWidget();
  mScenarioView->setColumnCount(3);
  mScenarioView->setColumnHidden(2, true);
  mScenarioView->setColumnWidth(0, 200);
  mScenarioView->setColumnWidth(1, 200);

  QStringList columnNames;
  columnNames << "Action"
              << "Feedback";
  mScenarioView->setHeaderLabels(columnNames);
  collapsedStates = new QMap<QString, bool>;
  rowLayout->addWidget(mScenarioView);

  formLayout->addRow(rowLayout);

  layout->addLayout(formLayout);

  setLayout(layout);

  // user input
  connect(mScenarioView, SIGNAL(itemCollapsed(QTreeWidgetItem *)), this,
          SLOT(handleItemCollapsed(QTreeWidgetItem *)));
  connect(mScenarioView, SIGNAL(itemExpanded(QTreeWidgetItem *)), this,
          SLOT(handleItemExpanded(QTreeWidgetItem *)));
}

void ScenarioView::onInitialize() {
  _node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  mBehaviorTreeSubscriber =
      _node->create_subscription<py_trees_ros_interfaces::msg::BehaviourTree>(
          "/scenario_execution/snapshots", 10,
          std::bind(&ScenarioView::behaviorTreeChanged, this, _1));
}

void ScenarioView::handleItemCollapsed(QTreeWidgetItem *collapsedItem) {
  collapsedStates->insert(collapsedItem->text(2), false);
}

void ScenarioView::handleItemExpanded(QTreeWidgetItem *expandedItem) {
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

bool ScenarioView::isNewTree(
    const py_trees_ros_interfaces::msg::BehaviourTree::SharedPtr previous,
    const py_trees_ros_interfaces::msg::BehaviourTree::SharedPtr current) const {

  bool isNew = false;
  if (previous != nullptr) {
    isNew = false;
    if (previous->behaviours.size() != current->behaviours.size()) {
      isNew = true;
    }
    for (size_t i = 0; (i < current->behaviours.size()) && !isNew; i++) {
      if (
          (current->behaviours.at(i).own_id.uuid != previous->behaviours.at(i).own_id.uuid) ||
          (current->behaviours.at(i).name != previous->behaviours.at(i).name)
      ){
        isNew = true;
      }
    }
  }
  else {
    isNew = true;
  }
  return isNew;
}

void ScenarioView::behaviorTreeChanged(
    const py_trees_ros_interfaces::msg::BehaviourTree::SharedPtr msg) {
  bool isNew = isNewTree(mPreviousMsg, msg);
  mPreviousMsg = msg;

  if (isNew) {
    QList<QTreeWidgetItem *> items;
    mScenarioView->clear();
    populateTree(items, msg);
    if (items.size() > 0) {
      items[0]->setBackground(0, Qt::white);
      items[0]->setBackground(1, Qt::white);
    }
    mScenarioView->insertTopLevelItems(0, items);

    // expand everything
    for (auto const &item : items) {
      mScenarioView->expandItem(item);
    }
  }
  else {
    QTreeWidgetItemIterator it(mScenarioView);
    while (*it) {
      auto msg_behavior = msg->behaviours.begin();
      while(msg_behavior < msg->behaviours.end())
      {
        if ((*it)->data(2, 0) == ConvertedBehavior::uuidToQString(msg_behavior->own_id.uuid)) {
          break;
        }
        msg_behavior++;
      }

      if (msg_behavior == msg->behaviours.end()) {
        qDebug() << "Cannot find corresponding behavior";
        break;
      }

      ConvertedBehavior elem(*msg_behavior);
      if ((*it)->data(2, 0) == elem.own_id) {
        setIcon(elem.status, *it);

        (*it)->setData(1,0, elem.message);
      }
      ++it;
    }
  }

  // set scenario result, if received
  if (mScenarioView->topLevelItem(0)) {
    
    for (auto it = msg->blackboard_on_visited_path.begin(); it != msg->blackboard_on_visited_path.end(); it++) {
        QString prefix = QString("/");
        prefix += mScenarioView->topLevelItem(0)->data(0,0).toString();
        prefix += "/";
        if ((prefix + "end" == QString::fromStdString(it->key)) && 
            ("True" == QString::fromStdString(it->value))) {
          mScenarioView->topLevelItem(0)->setBackground(0, QColor(180,255,180));
          mScenarioView->topLevelItem(0)->setBackground(1, QColor(180,255,180));
        }
        if ((prefix + "fail" == QString::fromStdString(it->key)) && 
            ("True" == QString::fromStdString(it->value))) {
          mScenarioView->topLevelItem(0)->setBackground(0, QColor(255,180,180));
          mScenarioView->topLevelItem(0)->setBackground(1, QColor(255,180,180));
        }
    }
  }
}

void ScenarioView::setIcon(int status, QTreeWidgetItem* item) const {
    if (status == 1) {
      item->setIcon(0, waitingIcon);
    } else if (status == 2) {
      item->setIcon(0, runningIcon);
    } else if (status == 3) {
      item->setIcon(0, successIcon);
    } else if (status == 4) {
      item->setIcon(0, failedIcon);
    }
}

void ScenarioView::populateTree(
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

    setIcon(behavior->status, items.last());
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

} // end namespace scenario_execution_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(scenario_execution_rviz::ScenarioView,
                       rviz_common::Panel)
