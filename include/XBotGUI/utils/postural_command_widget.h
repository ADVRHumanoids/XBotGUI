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
#ifndef XBOTGUI_POSTURAL_COMMAND_WIDGET_H
#define XBOTGUI_POSTURAL_COMMAND_WIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QHBoxLayout>
#include <QSlider>
#include <QLabel>
#include <QComboBox>
#include <string>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <XCM/JointStateAdvr.h>
#include "XBotGUI/utils/command_widget.h"
#include "XBotGUI/print_utils.h"
#include <urdf_parser/urdf_parser.h>

#define RAD2DEG 180.0/3.1415
#define DEG2RAD 3.1415/180.0

namespace XBot
{
namespace widgets
{
class postural_command_widget: public command_widget
{
Q_OBJECT
public:
	postural_command_widget(boost::shared_ptr<urdf::ModelInterface const> urdf_);
	~postural_command_widget();

private Q_SLOTS:
	void on_enable_button_clicked();
	void slider_slot();
	void on_minus_button_clicked();
	void on_plus_button_clicked();
	void on_joint_combo_changed();

private:
	ros::NodeHandle nh;

	QLabel title;
	QLabel min;
	QLabel max;
	QLabel current;
	QSlider slider;
	QPushButton minus_button;
	QPushButton plus_button;

	boost::shared_ptr<urdf::ModelInterface const> urdf;
	std::map<std::string,int> joint_indeces;
	std::map<std::string,double> joint_min;
	std::map<std::string,double> joint_max;
	std::mutex joint_mutex;
	ros::Publisher pub;
	sensor_msgs::JointState joint_states_cmd;
	ros::Subscriber sub;
	void joint_states_callback(const XCM::JointStateAdvr& joint_states);

	QComboBox joint_combo;
	QPushButton toggle_button;
	bool control_active=false;
	bool first_time_callback=true;

	QHBoxLayout _1st_layout;
	QHBoxLayout _2nd_layout;
	QVBoxLayout _3rd_layout;
	QHBoxLayout _4th_layout;
	QHBoxLayout _5th_layout;
	QVBoxLayout main_layout;
};
}
}

#endif