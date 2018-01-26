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

#include <XBotGUI/utils/postural_command_widget.h>

XBot::widgets::postural_command_widget::postural_command_widget(boost::shared_ptr<urdf::ModelInterface const> urdf_): urdf(urdf_)
{
    pub = nh.advertise<sensor_msgs::JointState>("joint_positions_desired",1);

    sub = nh.subscribe("/xbotcore/walkman/joint_states",1,&postural_command_widget::joint_states_callback,this);
    
    title.setText("-");
    slider.setOrientation(Qt::Horizontal);
    slider.setTickInterval(0.02);
    slider.setTickPosition(QSlider::NoTicks);
    slider.setMinimum(1);
    slider.setMaximum(1);
    slider.setValue(0);
    minus_button.setFixedSize(30,30);
    minus_button.setText("-");
    plus_button.setFixedSize(30,30);
    plus_button.setText("+");
    current.setFixedSize(70,40);
    current.setText(QString::number(0.0,'f',2));
    min.setFixedSize(70,40);
    min.setText(QString::number(0,'f',2));
    min.setFrameStyle(QFrame::Box);
    max.setFixedSize(70,40);
    max.setText(QString::number(0,'f',2));
    max.setFrameStyle(QFrame::Box);
    
    slider.setEnabled(false);
    minus_button.setEnabled(false);
    plus_button.setEnabled(false);
    toggle_button.setCheckable(true);
    toggle_button.setText("Postural Command waiting for joint state");
    toggle_button.setEnabled(false);
    
    _1st_layout.addWidget(&minus_button);
    _1st_layout.addWidget(&slider);
    _1st_layout.addWidget(&plus_button);
    _2nd_layout.addWidget(&min);
    _2nd_layout.addWidget(&current);
    _2nd_layout.addWidget(&max);
    _3rd_layout.addLayout(&_1st_layout);
    _3rd_layout.addLayout(&_2nd_layout);
    _4th_layout.addWidget(&title);
    _4th_layout.addLayout(&_3rd_layout);
    _5th_layout.addWidget(&joint_combo);
    _5th_layout.addWidget(&toggle_button);

    main_layout.addLayout(&_4th_layout);
    main_layout.addLayout(&_5th_layout);
    main_layout.addWidget(&toggle_button);

    connect(&slider, SIGNAL(valueChanged(int)), this, SLOT(slider_slot()));
    connect(&toggle_button, SIGNAL(clicked(bool)), this, SLOT(on_enable_button_clicked()));
    connect(&minus_button, SIGNAL(clicked(bool)), this, SLOT(on_minus_button_clicked()));
    connect(&plus_button, SIGNAL(clicked(bool)), this, SLOT(on_plus_button_clicked()));
    connect(&joint_combo, SIGNAL(currentIndexChanged(int)), this, SLOT(on_joint_combo_changed()));

    setLayout(&main_layout);

    control_active.store(false);
}

void XBot::widgets::postural_command_widget::on_joint_combo_changed()
{
    slider.setValue(joint_states_cmd.position.at(joint_indeces.at(joint_combo.currentText().toStdString()))*RAD2DEG);
    title.setText(joint_combo.currentText());
    slider.setMinimum(joint_min.at(joint_combo.currentText().toStdString()));
    slider.setMaximum(joint_max.at(joint_combo.currentText().toStdString()));
    min.setText(QString::number(joint_min.at(joint_combo.currentText().toStdString()),'f',2));
    max.setText(QString::number(joint_max.at(joint_combo.currentText().toStdString()),'f',2));
}

void XBot::widgets::postural_command_widget::on_enable_button_clicked()
{
    if(toggle_button.isChecked())
    {
        toggle_button.setText("DISABLE Postural Command");
	slider.setEnabled(true);
	minus_button.setEnabled(true);
	plus_button.setEnabled(true);

	control_active.store(true);

	pub.publish(joint_states_cmd);
    }
    else
    {
	toggle_button.setText("ENABLE Postural Command");
	slider.setEnabled(false);
	minus_button.setEnabled(false);
	plus_button.setEnabled(false);

	control_active.store(false);
    }
}

void XBot::widgets::postural_command_widget::on_minus_button_clicked()
{
    slider.setValue(slider.value()-5.0);
}

void XBot::widgets::postural_command_widget::on_plus_button_clicked()
{
    slider.setValue(slider.value()+5.0);
}

void XBot::widgets::postural_command_widget::slider_slot()
{
    current.setText(QString::number(slider.value(),'f',2));

    if(!control_active.load()) return;

    joint_states_cmd.position.at(joint_indeces.at(joint_combo.currentText().toStdString())) = slider.value()*DEG2RAD;
    pub.publish(joint_states_cmd);
}

void XBot::widgets::postural_command_widget::joint_states_callback(const XBotCore::JointStateAdvr& joint_states)
{
    if(control_active.load()) return;

    if(first_time_callback)
    {
	for(int i=0;i<joint_states.name.size();i++)
	{
	    joint_states_cmd.name.push_back(joint_states.name.at(i));
	    joint_states_cmd.position.push_back(0.0);

	    joint_indeces[joint_states.name.at(i)] = i;
	    joint_min[joint_states.name.at(i)] = urdf->getJoint(joint_states.name.at(i))->limits->lower*RAD2DEG;
	    joint_max[joint_states.name.at(i)] = urdf->getJoint(joint_states.name.at(i))->limits->upper*RAD2DEG;

	    joint_combo.addItem(QString::fromStdString(joint_states.name.at(i)));
	}

	first_time_callback=false;
	toggle_button.setEnabled(true);
	toggle_button.setText("ENABLE Postural Command");
    }

    for(int i=0;i<joint_states.motor_position.size();i++)
    {
	if(joint_indeces.at(joint_combo.currentText().toStdString())==i)
	{
	    slider.setValue(joint_states_cmd.position.at(i)*RAD2DEG);
	}

	joint_states_cmd.position.at(i) = joint_states.motor_position.at(i);
    }
}

XBot::widgets::postural_command_widget::~postural_command_widget()
{

}