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
#ifndef XBOTGUI_LOCOMOTION_COMMAND_WIDGET_H
#define XBOTGUI_LOCOMOTION_COMMAND_WIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QLabel>
#include <string>
#include <thread>
#include <atomic>
#include <ros/ros.h>
#include <ros/service.h>
#include <ADVR_ROS/advr_locomotion.h>
#include "XBotGUI/utils/command_widget.h"

namespace XBot
{
namespace widgets
{
class locomotion_command_widget: public command_widget
{
Q_OBJECT
public:
	locomotion_command_widget(std::string service_name_);

private Q_SLOTS:
	void on_walk_forward_button_clicked();
	void on_walk_backward_button_clicked();
	void on_walk_left_button_clicked();
	void on_walk_right_button_clicked();
	void on_turn_left_button_clicked();
	void on_turn_right_button_clicked();

private:
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::ServiceClient locomotion_client;
	void service_thread_body();
	std::atomic_bool thread_waiting;

	QFrame top_frame;
	QFrame mid_frame;
	QFrame bottom_frame;

	QLabel step_label;
	QLineEdit step_edit;
	QHBoxLayout step_layout;

	QPushButton walk_forward_button;
	QPushButton walk_backward_button;
	QPushButton walk_left_button;
	QPushButton walk_right_button;
	QLineEdit walk_edit;
	QGridLayout walk_layout;
	
	QPushButton turn_left_button;
	QPushButton turn_right_button;
	QLineEdit turn_edit;
	QVBoxLayout turn_layout;
	
	QHBoxLayout commands_layout;
	QLabel units_label;
	QVBoxLayout main_layout;

	std::string service_name;
	
	std::string command;
	void call_service();
};
}
}

#endif