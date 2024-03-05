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
#include "indicator_widget.h"
#include <QPainter>

namespace scenario_execution_rviz {

IndicatorWidget::IndicatorWidget(QWidget *parent)
    : QWidget(parent), mCurrentState(IndicatorWidget::State::Stopped) {
  setFixedSize(18, 18);
}

void IndicatorWidget::paintEvent(QPaintEvent *event) {
  (void)event;
  QPainter painter(this);
  painter.setPen(Qt::darkGray);
  if (mCurrentState == IndicatorWidget::State::Stopped) {
    painter.setBrush(QBrush(Qt::lightGray, Qt::SolidPattern));
  } else if (mCurrentState == IndicatorWidget::State::Running) {
    painter.setBrush(QBrush(Qt::green, Qt::SolidPattern));
  } else if (mCurrentState == IndicatorWidget::State::Error) {
    painter.setBrush(QBrush(Qt::red, Qt::SolidPattern));
  } else if ((mCurrentState == IndicatorWidget::State::Starting) ||
             (mCurrentState == IndicatorWidget::State::ShuttingDown)) {
    painter.setBrush(QBrush(Qt::yellow, Qt::SolidPattern));
  } else {
    painter.setBrush(QBrush(Qt::black, Qt::SolidPattern));
  }
  painter.drawEllipse(1, 1, 16, 16);
}

void IndicatorWidget::setState(IndicatorWidget::State state) {
  mCurrentState = state;
  repaint();
}

} // end namespace scenario_execution_rviz
