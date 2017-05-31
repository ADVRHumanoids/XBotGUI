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

XBot::widgets::joint::joint(std::string name_, boost::shared_ptr <const urdf::Joint > URDFjoint_, XBot::ControlMode control_mode_): QFrame()
{
    bool idle = false;
    name = name_;
    URDFjoint = URDFjoint_;
    control_mode = control_mode_;
    title.setText(QString::fromStdString(name));

    QPalette palette;
    palette.setColor(QPalette::Foreground,Qt::black);
    
    slider.setOrientation(Qt::Horizontal);
    slider.setTickInterval(0.02);
    slider.setTickPosition(QSlider::NoTicks);
    slider.setMinimum(URDFjoint->limits->lower*RAD2DEG);
    slider.setMaximum(URDFjoint->limits->upper*RAD2DEG);
    slider.setValue(0);

    if(control_mode.isPositionEnabled())
    {
	current.label.setText(QString::number(slider.value(),'f',2));
	min.label.setText(QString::number(URDFjoint->limits->lower*RAD2DEG,'f',2));
	max.label.setText(QString::number(URDFjoint->limits->upper*RAD2DEG,'f',2));
	control_mode_label.setText(" - Position [deg]");
    }
    else if(control_mode.isVelocityEnabled())
    {
	current.label.setText(QString::number(slider.value(),'f',2));
	min.label.setText(QString::number(-(URDFjoint->limits->velocity*RAD2DEG),'f',2));
	max.label.setText(QString::number(URDFjoint->limits->velocity*RAD2DEG,'f',2));
	control_mode_label.setText(" - Velocity [deg/s]");
    }
    else if(control_mode.isEffortEnabled())
    {
	current.label.setText(QString::number(slider.value(),'f',2));
	min.label.setText(QString::number(-(URDFjoint->limits->effort),'f',2));
	max.label.setText(QString::number(URDFjoint->limits->effort,'f',2));
	control_mode_label.setText(" - Torque [Nm]");
    }
    else if(control_mode.getName()=="IDLE")
    {
	current.label.setText(QString::number(slider.value(),'f',2));
	min.label.setText(QString::number(URDFjoint->limits->lower*RAD2DEG,'f',2));
	max.label.setText(QString::number(URDFjoint->limits->upper*RAD2DEG,'f',2));
	control_mode_label.setText("IDLE");
	idle=true;
    }
    
    current.setFixedSize(70,40);
    min.setFixedSize(70,40);
    min.setFrameStyle(QFrame::Box);
    min.setPalette(palette);
    max.setFixedSize(70,40);
    max.setFrameStyle(QFrame::Box);
    max.setPalette(palette);
    control_mode_label.setPalette(palette);

    labels_layout.addWidget(&min,Qt::AlignLeft);
    labels_layout.addWidget(&current,Qt::AnchorHorizontalCenter);
    labels_layout.addWidget(&max,Qt::AlignRight);

    main_layout.addWidget(&title);
    main_layout.addWidget(&slider);
    main_layout.addLayout(&labels_layout);
    main_layout.addWidget(&control_mode_label);
    
    slider.setEnabled(!idle);

    setFrameStyle(QFrame::Box);
    setAutoFillBackground(true);
    palette.setColor(QPalette::Background, Qt::lightGray);
    setPalette(palette);

    setLayout(&main_layout);
    setFixedSize(240,210);

    connect(&slider, SIGNAL(valueChanged(int)), this, SLOT(slider_slot()));
    connect(&slider, SIGNAL(actionTriggered(int)), this, SLOT(slider_action()));
    
    sense_timer.setSingleShot(true);
    connect(&sense_timer, SIGNAL(timeout()), this, SLOT(timer_slot()));
    
    disable_sense.store(false);
}

void XBot::widgets::joint::timer_slot()
{
    disable_sense.store(false);
}

void XBot::widgets::joint::set(double q_sense)
{
    if(disable_sense.load()) return;

    slider.setValue(q_sense*RAD2DEG);
    current.label.setText(QString::number(slider.value(),'f',2));

    if(!initialized)
    {
        slider_action();
	initialized = true;
    }
}

void XBot::widgets::joint::get(double& q_move)
{
    q_move = desired_q.load()*DEG2RAD;
}

void XBot::widgets::joint::slider_action()
{
    desired_q.store(slider.value());

    if(!sense_timer.isActive())
    {
        disable_sense.store(true);
	sense_timer.start(2000);
    }
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