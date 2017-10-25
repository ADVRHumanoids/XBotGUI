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

#include <XBotGUI/utils/gaze_command_widget.h>

XBot::widgets::gaze_command_widget::gaze_command_widget()
{
    pub = nh.advertise<sensor_msgs::JointState>("joint_positions_desired",1);

    sub = nh.subscribe("/xbotcore/bigman/joint_states",1,&gaze_command_widget::joint_states_callback,this);

    pitch_title.setText("Pitch: ");
    pitch_slider.setOrientation(Qt::Horizontal);
    pitch_slider.setTickInterval(0.02);
    pitch_slider.setTickPosition(QSlider::NoTicks);
    pitch_slider.setMinimum(PITCH_MIN);
    pitch_slider.setMaximum(PITCH_MAX);
    pitch_slider.setValue(0);
    pitch_minus_button.setFixedSize(30,30);
    pitch_minus_button.setText("-");
    pitch_plus_button.setFixedSize(30,30);
    pitch_plus_button.setText("+");
    pitch_current.setFixedSize(70,40);
    pitch_current.setText(QString::number(0.0,'f',2));
    pitch_min.setFixedSize(70,40);
    pitch_min.setText(QString::number(PITCH_MIN,'f',2));
    pitch_min.setFrameStyle(QFrame::Box);
    pitch_max.setFixedSize(70,40);
    pitch_max.setText(QString::number(PITCH_MAX,'f',2));
    pitch_max.setFrameStyle(QFrame::Box);
    
    yaw_title.setText("Yaw: ");
    yaw_slider.setOrientation(Qt::Horizontal);
    yaw_slider.setTickInterval(0.02);
    yaw_slider.setTickPosition(QSlider::NoTicks);
    yaw_slider.setMinimum(YAW_MIN);
    yaw_slider.setMaximum(YAW_MAX);
    yaw_slider.setValue(0);
    yaw_minus_button.setFixedSize(30,30);
    yaw_minus_button.setText("-");
    yaw_plus_button.setFixedSize(30,30);
    yaw_plus_button.setText("+");
    yaw_current.setFixedSize(70,40);
    yaw_current.setText(QString::number(0.0,'f',2));
    yaw_min.setFixedSize(70,40);
    yaw_min.setText(QString::number(YAW_MIN,'f',2));
    yaw_min.setFrameStyle(QFrame::Box);
    yaw_max.setFixedSize(70,40);
    yaw_max.setText(QString::number(YAW_MAX,'f',2));
    yaw_max.setFrameStyle(QFrame::Box);

    pitch_slider.setEnabled(false);
    yaw_slider.setEnabled(false);
    toggle_button.setCheckable(true);
    toggle_button.setText("Gaze Control waiting for joint state");
    toggle_button.setEnabled(false);

    pitch_1st_layout.addWidget(&pitch_minus_button);
    pitch_1st_layout.addWidget(&pitch_slider);
    pitch_1st_layout.addWidget(&pitch_plus_button);
    pitch_2nd_layout.addWidget(&pitch_min);
    pitch_2nd_layout.addWidget(&pitch_current);
    pitch_2nd_layout.addWidget(&pitch_max);
    pitch_3rd_layout.addLayout(&pitch_1st_layout);
    pitch_3rd_layout.addLayout(&pitch_2nd_layout);
    pitch_4th_layout.addWidget(&pitch_title);
    pitch_4th_layout.addLayout(&pitch_3rd_layout);
    
    yaw_1st_layout.addWidget(&yaw_minus_button);
    yaw_1st_layout.addWidget(&yaw_slider);
    yaw_1st_layout.addWidget(&yaw_plus_button);
    yaw_2nd_layout.addWidget(&yaw_min);
    yaw_2nd_layout.addWidget(&yaw_current);
    yaw_2nd_layout.addWidget(&yaw_max);
    yaw_3rd_layout.addLayout(&yaw_1st_layout);
    yaw_3rd_layout.addLayout(&yaw_2nd_layout);
    yaw_4th_layout.addWidget(&yaw_title);
    yaw_4th_layout.addLayout(&yaw_3rd_layout);

    main_layout.addLayout(&pitch_4th_layout);
    main_layout.addLayout(&yaw_4th_layout);
    main_layout.addWidget(&toggle_button);

    connect(&pitch_slider, SIGNAL(valueChanged(int)), this, SLOT(pitch_slider_slot()));
    connect(&yaw_slider, SIGNAL(valueChanged(int)), this, SLOT(yaw_slider_slot()));
    connect(&toggle_button, SIGNAL(clicked(bool)), this, SLOT(on_enable_button_clicked()));
    connect(&yaw_minus_button, SIGNAL(clicked(bool)), this, SLOT(on_yaw_minus_button_clicked()));
    connect(&yaw_plus_button, SIGNAL(clicked(bool)), this, SLOT(on_yaw_plus_button_clicked()));
    connect(&pitch_minus_button, SIGNAL(clicked(bool)), this, SLOT(on_pitch_minus_button_clicked()));
    connect(&pitch_plus_button, SIGNAL(clicked(bool)), this, SLOT(on_pitch_plus_button_clicked()));

    setLayout(&main_layout);
}

void XBot::widgets::gaze_command_widget::on_enable_button_clicked()
{
    if(toggle_button.isChecked())
    {
        toggle_button.setText("DISABLE Gaze Control");
	pitch_slider.setEnabled(true);
	yaw_slider.setEnabled(true);

	joint_mutex.lock();
	control_active=true;
	joint_mutex.unlock();

	pub.publish(joint_states_cmd);
    }
    else
    {
	toggle_button.setText("ENABLE Gaze Control");
	pitch_slider.setEnabled(false);
	yaw_slider.setEnabled(false);

	joint_mutex.lock();
	control_active=false;
	joint_mutex.unlock();
    }
}

void XBot::widgets::gaze_command_widget::on_pitch_minus_button_clicked()
{
    pitch_slider.setValue(pitch_slider.value()-5.0);
}

void XBot::widgets::gaze_command_widget::on_pitch_plus_button_clicked()
{
    pitch_slider.setValue(pitch_slider.value()+5.0);
}

void XBot::widgets::gaze_command_widget::on_yaw_minus_button_clicked()
{
    yaw_slider.setValue(yaw_slider.value()-5.0);
}

void XBot::widgets::gaze_command_widget::on_yaw_plus_button_clicked()
{
    yaw_slider.setValue(yaw_slider.value()+5.0);
}

void XBot::widgets::gaze_command_widget::pitch_slider_slot()
{
    pitch_current.setText(QString::number(pitch_slider.value(),'f',2));
    joint_states_cmd.position.at(pitch_index) = pitch_slider.value()*DEG2RAD;
    pub.publish(joint_states_cmd);
}

void XBot::widgets::gaze_command_widget::yaw_slider_slot()
{
    yaw_current.setText(QString::number(yaw_slider.value(),'f',2));
    joint_states_cmd.position.at(yaw_index) = yaw_slider.value()*DEG2RAD;
    pub.publish(joint_states_cmd);
}

void XBot::widgets::gaze_command_widget::joint_states_callback(const XCM::JointStateAdvr& joint_states)
{
    joint_mutex.lock();
    
    if(control_active)
    {
	joint_mutex.unlock();
	return;
    }

    if(first_time_callback)
    {
	for(int i=0;i<joint_states.name.size();i++)
	{
	    joint_states_cmd.name.push_back(joint_states.name.at(i));
	    joint_states_cmd.position.push_back(0.0);

	    if(joint_states.name.at(i)=="NeckYawj") pitch_index=i;
	    if(joint_states.name.at(i)=="NeckPitchj") yaw_index=i;
	}
	
	if(pitch_index==-1 || yaw_index==-1)
	{
	    std::cout<<red_string("[ ERROR: neck joints are not in the robot status ]")<<std::endl;
	    joint_mutex.unlock();
	    return;
	}

	first_time_callback=false;
	toggle_button.setEnabled(true);
	toggle_button.setText("ENABLE Gaze Control");
    }

    for(int i=0;i<joint_states.motor_position.size();i++)
    {
	joint_states_cmd.position.at(i) = joint_states.motor_position.at(i);

	if(i==pitch_index)
	{
	    pitch_slider.setValue(joint_states_cmd.position.at(i)*RAD2DEG);
	}
	if(i==yaw_index)
	{
	    yaw_slider.setValue(joint_states_cmd.position.at(i)*RAD2DEG);
	}
    }
    
    joint_mutex.unlock();
}

XBot::widgets::gaze_command_widget::~gaze_command_widget()
{

}