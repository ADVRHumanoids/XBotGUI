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

#include "XBotGUI/utils/manipulation_map_utils_widget.h"
#include "XBotGUI/print_utils.h"
#include <iostream>
#include <cstdlib>

XBot::widgets::manipulation_map_utils_widget::manipulation_map_utils_widget()
{
    switch_button.setCheckable(true);
    switch_button.setText("Show Manipulation Map");

    main_layout.addWidget(&switch_button);

    connect(&switch_button,SIGNAL(clicked()),this,SLOT(on_switch_button_clicked()));

    setLayout(&main_layout);

    std::map<std::string,std::string> properties;

    properties["Marker Topic"] = "/manipulation_map";
    displays.push_back(std::make_tuple("manipulation_map","rviz/Marker",properties));

	visualization_msgs::Marker marker;
    marker.header.frame_id="/world_odom";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = 0.04;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
	marker.pose.position.x = 0.78;
	marker.pose.position.y = 0.48;
	marker.pose.position.z = 1.05;
	marker.pose.orientation.w=1;
    marker.color.a = 0.5;
    marker.color.g = 1;

	markers.push_back(marker);

	marker.id = 1;
	marker.scale.x = 0.08;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
	marker.pose.position.x = 0.78;
	marker.pose.position.y = 0.50;

	markers.push_back(marker);
	
	marker.id = 2;
	marker.scale.x = 0.1;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
	marker.pose.position.x = 0.79;
	marker.pose.position.y = 0.52;

	markers.push_back(marker);

	marker.id = 3;
	marker.scale.x = 0.12;
    marker.scale.y = 0.07;
    marker.scale.z = 0.02;
	marker.pose.position.x = 0.84;
	marker.pose.position.y = 0.565;

	markers.push_back(marker);

	marker.id = 4;
	marker.scale.x = 0.08;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
	marker.pose.position.x = 0.84;
	marker.pose.position.y = 0.61;

	markers.push_back(marker);

	marker.id = 5;
	marker.scale.x = 0.04;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
	marker.pose.position.x = 0.84;
	marker.pose.position.y = 0.63;

	markers.push_back(marker);
 
    pub = nh.advertise<visualization_msgs::Marker>("/manipulation_map",1);
}

void XBot::widgets::manipulation_map_utils_widget::on_switch_button_clicked()
{
    if(switch_button.isChecked())
    {
	switch_button.setText("Hide Manipulation Map");

    for(auto m:markers)
	{
		m.action = visualization_msgs::Marker::ADD;
		pub.publish(m);
	}
	std::cout<<green_string("[ UTIL manipulation_map ") + yellow_string("STARTED") + green_string(" ]")<<std::endl;
    }
    else
    {
	switch_button.setText("Show Manipulation Map");

    for(auto m:markers)
	{
		m.action = visualization_msgs::Marker::DELETE;
		pub.publish(m);
	}
	std::cout<<green_string("[ UTIL manipulation_map ") + yellow_string("STARTED") + green_string(" ]")<<std::endl;

	std::cout<<green_string("[ UTIL manipulation ") + yellow_string("STOPPED") + green_string(" ]")<<std::endl;
    }
}

XBot::widgets::manipulation_map_utils_widget::~manipulation_map_utils_widget()
{

}
