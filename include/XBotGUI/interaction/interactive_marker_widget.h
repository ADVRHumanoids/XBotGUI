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
#include <QBoxLayout>
#include <QPushButton>
#include "XBotGUI/utils/interactive_markers_handler.h"

namespace XBot
{
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

private:
    rviz::Tool* interactive_tool;
    rviz::ToolManager* tool_manager;
    ros::NodeHandle nh;
    interactive_markers_handler im_handler;
    ros::Publisher marker_pub;
    visualization_msgs::Marker marker;

    QPushButton interactive_tool_button;
    QPushButton publish_button;

    QHBoxLayout buttons_layout;
    QVBoxLayout main_layout;

};
};
};

#endif