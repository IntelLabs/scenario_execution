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

#include <QWidget>

namespace scenario_execution_rviz {

class IndicatorWidget : public QWidget {
  Q_OBJECT
public:
  enum class State { Stopped, Starting, Running, ShuttingDown, Error };

  IndicatorWidget(QWidget *parent = 0);

  virtual void paintEvent(QPaintEvent *event) override;

  void setState(IndicatorWidget::State state);

private:
  State mCurrentState{State::Stopped};
};

} // end namespace scenario_execution_rviz
