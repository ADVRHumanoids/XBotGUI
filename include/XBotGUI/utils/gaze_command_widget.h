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
#ifndef XBOTGUI_GAZE_COMMAND_WIDGET_H
#define XBOTGUI_GAZE_COMMAND_WIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QHBoxLayout>
#include <QSlider>
#include <QLabel>
#include <string>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <XCM/JointStateAdvr.h>
#include "XBotGUI/utils/command_widget.h"
#include "XBotGUI/print_utils.h"

#define PITCH_MIN -15.0
#define PITCH_MAX 60.0
#define YAW_MIN -35.0
#define YAW_MAX 35.0

#define RAD2DEG 180.0/3.1415
#define DEG2RAD 3.1415/180.0

namespace XBot
{
namespace widgets
{
class gaze_command_widget: public command_widget
{
Q_OBJECT
public:
	gaze_command_widget();
	~gaze_command_widget();

private Q_SLOTS:
	void on_enable_button_clicked();
	void pitch_slider_slot();
	void yaw_slider_slot();
	void on_pitch_minus_button_clicked();
	void on_pitch_plus_button_clicked();
	void on_yaw_minus_button_clicked();
	void on_yaw_plus_button_clicked();

private:
	ros::NodeHandle nh;
	
	QLabel pitch_title;
	QLabel pitch_min;
	QLabel pitch_max;
	QLabel pitch_current;
	QSlider pitch_slider;
	QPushButton pitch_minus_button;
	QPushButton pitch_plus_button;

	QLabel yaw_title;
	QLabel yaw_min;
	QLabel yaw_max;
	QLabel yaw_current;
	QSlider yaw_slider;
	QPushButton yaw_minus_button;
	QPushButton yaw_plus_button;

	std::mutex joint_mutex;
	ros::Publisher pub;
	sensor_msgs::JointState joint_states_cmd;
	ros::Subscriber sub;
	void joint_states_callback(const XCM::JointStateAdvr& joint_states);

	QPushButton toggle_button;
	bool control_active=false;
	bool first_time_callback=true;
	int pitch_index=-1;
	int yaw_index=-1;

	QHBoxLayout pitch_1st_layout;
	QHBoxLayout pitch_2nd_layout;
	QVBoxLayout pitch_3rd_layout;
	QHBoxLayout pitch_4th_layout;
	QHBoxLayout yaw_1st_layout;
	QHBoxLayout yaw_2nd_layout;
	QVBoxLayout yaw_3rd_layout;
	QHBoxLayout yaw_4th_layout;
	QVBoxLayout main_layout;
};
}
}

#endif