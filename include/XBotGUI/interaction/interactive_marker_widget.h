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
#include <rviz/tool_manager.h>
#include <visualization_msgs/Marker.h>
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
    geometry_msgs::Vector3 scale;
    visualization_msgs::Marker::Type::_type_type type;
    std::string mesh_name;
};

namespace widgets
{
class im_widget: public QWidget
{
Q_OBJECT
public:
    im_widget(rviz::ToolManager* tool_manager_, std::string name, int index);
    ~im_widget();

private Q_SLOTS:
    void on_publish_button_clicked();
    void on_interactive_tool_button_clicked();
    void on_coords_changed(int id);
    void on_object_combo_changed();
    void on_scale_changed(int id);

private:
    rviz::Tool* interactive_tool;
    rviz::ToolManager* tool_manager;
    ros::NodeHandle nh;
    interactive_markers_handler im_handler;
    ros::Publisher marker_pub;
    visualization_msgs::Marker marker;
    ros::Subscriber im_sub;
    void im_callback(const visualization_msgs::InteractiveMarkerFeedback& feedback);
    void update_coords();
    std::atomic_bool changing_coords;

    QPushButton interactive_tool_button;
    QPushButton publish_button;

    QSignalMapper coord_mapper;
    std::map<int, label_lineedit*> coords_widgets;

    QGridLayout coords_layout;
    QHBoxLayout buttons_layout;
    QVBoxLayout main_layout;

    std::map<std::string,object_properties*> objects;
    QComboBox object_combo;
    void generate_objects();
    void update_scale();
    QHBoxLayout scale_layout;
    QLabel scale_label;
    std::map<int, label_lineedit*> scale_widgets;
    QSignalMapper scale_mapper;
    void load_object_params();
    
    ros::Publisher debug_pub;
};
};
};

#endif