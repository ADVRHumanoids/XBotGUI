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

#ifndef XBOTGUI_IM_WIDGET_H
#define XBOTGUI_IM_WIDGET_H

#include <ros/ros.h>
#include <ros/service.h>
#include <ADVR_ROS/im_pose.h>
#include <rviz/tool_manager.h>
#include <rviz/properties/property.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>
#include <QBoxLayout>
#include <QComboBox>
#include <QPushButton>
#include <QSignalMapper>
#include "XBotGUI/utils/interactive_markers_handler.h"
#include "XBotGUI/utils/Label_LineEdit.h" //NOTE: mixing CamelCase and under_score because reasons
#include <atomic>

namespace XBot
{

class object_properties
{
public:
    object_properties();

    std::string name;
    int id;
    geometry_msgs::Vector3 scale;
    geometry_msgs::Pose pose;
    visualization_msgs::Marker::Type::_type_type type;
    std::string mesh_name;
};

namespace widgets
{
class im_widget: public QWidget
{
Q_OBJECT
public:
    im_widget(rviz::ToolManager* tool_manager_, std::string name_, std::map<std::string,object_properties> objects_);
    ~im_widget();

private Q_SLOTS:
    void on_publish_button_clicked();
    void on_position_by_click_button_clicked();
    void on_coords_changed(int id);
    void on_object_combo_changed();
    void on_scale_changed(int id);

private:
    rviz::ToolManager* tool_manager;
    std::string name;
    ros::NodeHandle nh;
    interactive_markers_handler* im_handler;
    std::map<int,int> combo_ids;
    ros::ServiceServer pose_service;
    bool pose_service_callback(ADVR_ROS::im_pose::Request &req, ADVR_ROS::im_pose::Response &res);
    ros::Publisher marker_pub;
    visualization_msgs::Marker marker;
    ros::Subscriber im_sub;
    void im_callback(const visualization_msgs::InteractiveMarkerFeedback& feedback);
    void update_coords();
    std::atomic_bool changing_coords;

    QPushButton publish_button;

    QPushButton position_by_click_button;
    void position_by_click_callback(const geometry_msgs::PointStamped& point);
    ros::Subscriber position_by_click_sub;
    rviz::Tool* click_tool;

    QSignalMapper coord_mapper;
    std::map<int, label_lineedit*> coords_widgets;

    QGridLayout coords_layout;
    QHBoxLayout buttons_layout;
    QVBoxLayout main_layout;

    std::map<std::string,object_properties> objects;
    QComboBox object_combo;
    void generate_objects(std::map<std::string,object_properties> objects_);
    void update_scale();
    QHBoxLayout scale_layout;
    QLabel scale_label;
    std::map<int, label_lineedit*> scale_widgets;
    QSignalMapper scale_mapper;
    void load_object_params();
    std::atomic_bool changing_scale;

};
};
};

#endif