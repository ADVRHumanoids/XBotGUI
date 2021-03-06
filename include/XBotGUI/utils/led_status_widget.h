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

#ifndef XBOTGUI_LED_STATUS_WIDGET_H
#define XBOTGUI_LED_STATUS_WIDGET_H

#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>

namespace XBot
{
namespace widgets
{
class module;

class led_status_widget: public QWidget
{
public:
    led_status_widget(std::string name, std::string topic);
    ~led_status_widget();

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    void status_callback(const std_msgs::String& color);

    QPixmap red_pixmap;
    QPixmap yellow_pixmap;
    QPixmap green_pixmap;

    QLabel led_label;
    QLabel name_label;
    QVBoxLayout main_layout;
};
};
};

#endif