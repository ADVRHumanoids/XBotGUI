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

#include "XBotGUI/utils/led_status_widget.h"

XBot::widgets::led_status_widget::led_status_widget(std::string name, std::string topic)
{
    sub = nh.subscribe(topic.c_str(),1,&led_status_widget::status_callback,this);

    name_label.setText(QString::fromStdString(name));

    std::string filename;
    filename = std::getenv("ROBOTOLOGY_ROOT") + std::string("/external/XBotGUI/resources/green_circle.png");
    green_pixmap.load(QString::fromStdString(filename));
    filename = std::getenv("ROBOTOLOGY_ROOT") + std::string("/external/XBotGUI/resources/yellow_circle.png");
    yellow_pixmap.load(QString::fromStdString(filename));
    filename = std::getenv("ROBOTOLOGY_ROOT") + std::string("/external/XBotGUI/resources/red_circle.png");
    red_pixmap.load(QString::fromStdString(filename));

    led_label.setPixmap(red_pixmap);
    led_label.setFixedSize(20,20);

    main_layout.addWidget(&led_label);
    main_layout.addWidget(&name_label);

    setFixedSize(100,60);

    setLayout(&main_layout);
}

void XBot::widgets::led_status_widget::status_callback(const std_msgs::String& status)
{
    if(status.data=="green")
    {
	led_label.setPixmap(green_pixmap);
    }
    if(status.data=="yellow")
    {
	led_label.setPixmap(yellow_pixmap);
    }
    if(status.data=="red")
    {
	led_label.setPixmap(red_pixmap);
    }
}

XBot::widgets::led_status_widget::~led_status_widget()
{
  
}