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
#ifndef XBOTGUI_GRASP_WIDGET_H
#define XBOTGUI_GRASP_WIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QHBoxLayout>
#include <QSlider>
#include <QLabel>
#include <QLineEdit>
#include <string>
#include <thread>
#include <atomic>
#include <ros/ros.h>
#include <ros/service.h>
#include <ADVR_ROS/advr_grasp_control_srv.h>
#include <XBotGUI/utils/command_widget.h>

namespace XBot
{
namespace widgets
{
class grasp_widget: public command_widget
{
Q_OBJECT
public:
	grasp_widget();

private Q_SLOTS:
	void on_grasp_button_clicked();
	void left_slider_slot();
	void right_slider_slot();

private:
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::ServiceClient grasp_client;
	ADVR_ROS::advr_grasp_control_srv srv;
	void service_thread_body();
	std::atomic_bool thread_waiting;

	QLabel left_label;
	QLabel right_label;
	QSlider left_slider;
	QHBoxLayout left_layout;
	
	QSlider right_slider;
	QLineEdit left_edit;
	QLineEdit right_edit;
	QHBoxLayout right_layout;
	
        QPushButton grasp_button;
	QVBoxLayout main_layout;
};
}
}

#endif