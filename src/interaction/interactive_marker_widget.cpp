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
#include "XBotGUI/interaction/interactive_marker_widget.h"

XBot::widgets::im_widget::im_widget(std::string name): QWidget(), im_handler(name+"_server",name+"_client")
{
   marker_pub = nh.advertise<visualization_msgs::Marker>(name+"_client",1);
   name = "Publish " + name;
   publish_button.setText(QString::fromStdString(name));

   marker.color.r=1;
   marker.color.a=1;
   marker.pose.orientation.w=1;
   marker.header.frame_id="/base_link";
   marker.type=visualization_msgs::Marker::CUBE;
   marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
   
   main_layout.addWidget(&publish_button);
   
   setLayout(&main_layout);
   
   connect(&publish_button, SIGNAL(clicked(bool)), this, SLOT(on_publish_button_clicked()));
}

void XBot::widgets::im_widget::on_publish_button_clicked()
{
    marker_pub.publish(marker);
}

XBot::widgets::im_widget::~im_widget()
{

}