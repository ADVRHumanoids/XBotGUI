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

XBot::widgets::im_widget::im_widget(rviz::ToolManager* tool_manager_, std::string name, int index): QWidget(), tool_manager(tool_manager_), im_handler(name+"_server",name+"_client")
{
   interactive_tool = tool_manager->addTool("rviz/Interact");

   marker_pub = nh.advertise<visualization_msgs::Marker>(name+"_client",1);
   publish_button.setText("Publish Marker");
   interactive_tool_button.setCheckable(true);
   interactive_tool_button.setText("Enable Interaction");

   marker.id = index;
   marker.color.g=1;
   marker.color.a=1;
   marker.pose.orientation.w=1;
   marker.header.frame_id="/base_link";
   marker.type=visualization_msgs::Marker::CUBE;
   marker.scale.x = marker.scale.y = marker.scale.z = 1.0;

   buttons_layout.addWidget(&publish_button);
   buttons_layout.addWidget(&interactive_tool_button);

   main_layout.addLayout(&buttons_layout);
   
   setLayout(&main_layout);
   
   connect(&publish_button, SIGNAL(clicked(bool)), this, SLOT(on_publish_button_clicked()));
   connect(&interactive_tool_button, SIGNAL(clicked(bool)), this, SLOT(on_interactive_tool_button_clicked()));
}

void XBot::widgets::im_widget::on_interactive_tool_button_clicked()
{
    if(interactive_tool_button.isChecked())
    {
	tool_manager->setCurrentTool(interactive_tool);
	interactive_tool_button.setText("Disable Interaction");
    }
    else
    {
	tool_manager->setCurrentTool(tool_manager->getDefaultTool());
	interactive_tool_button.setText("Enable Interaction");
    }
}

void XBot::widgets::im_widget::on_publish_button_clicked()
{
    marker_pub.publish(marker);
}

XBot::widgets::im_widget::~im_widget()
{

}