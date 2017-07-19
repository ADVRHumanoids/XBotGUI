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
#ifndef XBOTGUI_STRING_COMMAND_WIDGET_H
#define XBOTGUI_STRING_COMMAND_WIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QHBoxLayout>
#include <string>
#include <thread>
#include <atomic>
#include <ros/ros.h>
#include <ros/service.h>
#include <XCM/cmd_service.h>
#include <XBotGUI/utils/command_widget.h>

namespace XBot
{
namespace widgets
{
class string_command_widget: public command_widget
{
Q_OBJECT
public:
	string_command_widget(std::string module_name_, std::string command_name_);

private Q_SLOTS:
	void on_cmd_button_clicked();

private:
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::ServiceClient cmd_client;
	void service_thread_body();
	std::atomic_bool thread_waiting;

        QPushButton cmd_button;
	QVBoxLayout main_layout;

	std::string module_name;
	std::string command_name;
};
}
}

#endif