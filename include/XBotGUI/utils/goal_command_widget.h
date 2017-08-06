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
#ifndef XBOTGUI_GOAL_COMMAND_WIDGET_H
#define XBOTGUI_GOAL_COMMAND_WIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QHBoxLayout>
#include <QSignalMapper>
#include <string>
#include <thread>
#include <atomic>
#include <ros/ros.h>
#include <ros/service.h>
#include "XBotGUI/utils/Label_LineEdit.h"
#include <rviz/tool_manager.h>
#include <rviz/properties/property.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include "XBotGUI/utils/command_widget.h"
#include "XBotGUI/print_utils.h"

namespace XBot
{
namespace widgets
{
class goal_command_widget: public command_widget
{
Q_OBJECT
public:
	goal_command_widget(rviz::ToolManager* tool_manager_, std::string topic_name_);
	void set_fixed_frame(std::string frame);

private Q_SLOTS:
	void on_select_goal_button_clicked();
	void on_show_goal_button_clicked();
	void on_send_goal_button_clicked();
	void on_coords_changed(int id);

private:
	ros::NodeHandle nh;
	tf::TransformListener tf_;
	ros::Publisher pub;
	ros::Subscriber sub;
	rviz::ToolManager* tool_manager;
	rviz::Tool* goal_tool;
	rviz::Tool* last_tool;
	void goal_callback(const geometry_msgs::PoseStamped& pose);
	geometry_msgs::PoseStamped last_pose;

	QPushButton select_goal_button;
	QPushButton show_goal_button;
	QPushButton send_goal_button;
	std::map<int, label_lineedit*> coords_widgets;
	QSignalMapper coord_mapper;
	QGridLayout coords_layout;
	std::atomic_bool changing_coords;
	QHBoxLayout c_layout;

	std::string topic_name;

	ros::Publisher marker_pub;
	visualization_msgs::Marker marker;
	void update_marker();
	void publish_marker();
	void update_coords(const geometry_msgs::Pose& pose);
};
}
}

#endif