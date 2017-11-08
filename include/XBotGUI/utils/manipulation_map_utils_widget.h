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

#ifndef XBOTGUI_MANIPULATION_MAP_UTILS_WIDGET_H
#define XBOTGUI_MANIPULATION_MAP_UTILS_WIDGET_H

#include <QBoxLayout>
#include <QPushButton>
#include <QWidget>
#include <QProcess>
#include <string>
#include <map>
#include <vector>
#include <tuple>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

typedef std::tuple<std::string, std::string, std::map< std::string, std::string >> display_property;

namespace XBot
{
namespace widgets
{

class manipulation_map_utils_widget: public QWidget
{
Q_OBJECT
public:
    manipulation_map_utils_widget();
    ~manipulation_map_utils_widget();

    std::vector<display_property> displays;

private Q_SLOTS:
    void on_switch_button_clicked();

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    std::vector<visualization_msgs::Marker> markers;

    QPushButton switch_button;

    QHBoxLayout main_layout;

};
};
};

#endif
