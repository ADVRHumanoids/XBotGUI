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

XBot::widgets::joint::joint(std::string name_, boost::shared_ptr<urdf::ModelInterface const> urdf): QWidget()
{
    name = name_;
    title.setText(QString::fromStdString(name));

    auto urdf_joint = urdf->getJoint(name);

    slider.setOrientation(Qt::Horizontal);
    slider.setTickInterval(0.02);
    slider.setTickPosition(QSlider::TicksBelow);
    slider.setMinimum(urdf_joint->limits->lower*RAD2DEG);
    slider.setMaximum(urdf_joint->limits->upper*RAD2DEG);
    slider.setValue(0);

    main_layout.addWidget(&title);
    main_layout.addWidget(&slider);

    setLayout(&main_layout);
    setStyleSheet("border: 1px solid black");

    connect(&slider, SIGNAL(valueChanged(int)), this, SLOT(slider_slot()));
}

void XBot::widgets::joint::slider_slot()
{
    std::cout<<name<<" : "<<slider.value()<<std::endl;
}

XBot::widgets::joint::~joint()
{

}