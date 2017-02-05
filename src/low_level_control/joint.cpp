/*
 * Copyright (C) 2017 Centro di Ricerca "E. Piaggio" (Universita' di Pisa), IIT-ADVR
 * Author: Alessandro Settimi, Arturo Laurenzi, Luca Muratore
 * email:  ale.settimi@gmail.com, arturo.laurenzi@iit.it, luca.muratore@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include "XBotGUI/low_level_control/joint.h"

XBot::widgets::joint::joint(std::string name_, boost::shared_ptr<urdf::ModelInterface const> urdf): QFrame()
{
    name = name_;
    title.setText(QString::fromStdString(name));

    auto urdf_joint = urdf->getJoint(name);

    QPalette palette;
    palette.setColor(QPalette::Foreground,Qt::black);
    
    slider.setOrientation(Qt::Horizontal);
    slider.setTickInterval(0.02);
    slider.setTickPosition(QSlider::NoTicks);
    slider.setMinimum(urdf_joint->limits->lower*RAD2DEG);
    slider.setMaximum(urdf_joint->limits->upper*RAD2DEG);
    slider.setValue(0);

    current.label.setText(QString::number(slider.value(),'f',2));
    current.setFixedSize(70,30);
    min.label.setText(QString::number(urdf_joint->limits->lower*RAD2DEG,'f',2));
    min.setFixedSize(70,30);
    min.setFrameStyle(QFrame::Box);
    min.setPalette(palette);
    max.label.setText(QString::number(urdf_joint->limits->upper*RAD2DEG,'f',2));
    max.setFixedSize(70,30);
    max.setFrameStyle(QFrame::Box);
    max.setPalette(palette);
    labels_layout.addWidget(&min,Qt::AlignLeft);
    labels_layout.addWidget(&current,Qt::AnchorHorizontalCenter);
    labels_layout.addWidget(&max,Qt::AlignRight);

    main_layout.addWidget(&title);
    main_layout.addWidget(&slider);
    main_layout.addLayout(&labels_layout);

    setFrameStyle(QFrame::Box);
    setAutoFillBackground(true);
    palette.setColor(QPalette::Background, Qt::lightGray);
    setPalette(palette);

    setLayout(&main_layout);
    setFixedSize(240,210);

    connect(&slider, SIGNAL(valueChanged(int)), this, SLOT(slider_slot()));
}

void XBot::widgets::joint::slider_slot()
{
    current.label.setText(QString::number(slider.value(),'f',2));
}

XBot::widgets::joint::~joint()
{

}

XBot::widgets::QBoxedLabel::QBoxedLabel(): QFrame()
{
    layout.addWidget(&label);
    setLayout(&layout);
}

XBot::widgets::QBoxedLabel::~QBoxedLabel()
{

}