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
#include <tf/transform_datatypes.h>

XBot::widgets::im_widget::im_widget(rviz::ToolManager* tool_manager_, std::string name, int index): QWidget(), tool_manager(tool_manager_), im_handler(name+"_server",name+"_client")
{
    changing_test.store(false);

   interactive_tool = tool_manager->addTool("rviz/Interact");

   marker_pub = nh.advertise<visualization_msgs::Marker>(name+"_client",1);
   publish_button.setText("Publish Marker");
   interactive_tool_button.setCheckable(true);
   interactive_tool_button.setText("Enable Interaction");
   im_sub = nh.subscribe(("/"+name+"_server/feedback").c_str(),1,&im_widget::im_callback,this);

   coords_widgets[0] = new label_lineedit("x:");
   coords_widgets[1] = new label_lineedit("y:");
   coords_widgets[2] = new label_lineedit("z:");
   coords_widgets[3] = new label_lineedit("R:");
   coords_widgets[4] = new label_lineedit("P:");
   coords_widgets[5] = new label_lineedit("Y:");

   for(int i=0;i<6;i++)
   {
      connect(&(coords_widgets.at(i)->edit), SIGNAL(textChanged(QString)),&coord_mapper, SLOT(map()));
      coord_mapper.setMapping(&(coords_widgets.at(i)->edit), i);
   }
   connect(&coord_mapper, SIGNAL(mapped(int)), this, SLOT(on_text_edit_changed(int))) ;

   marker.id = index;
   marker.color.g=1;
   marker.color.a=1;
   marker.pose.orientation.w=1;
   marker.header.frame_id="/base_link";
   marker.type=visualization_msgs::Marker::CUBE;
   marker.scale.x = marker.scale.y = marker.scale.z = 1.0;

   buttons_layout.addWidget(&publish_button);
   buttons_layout.addWidget(&interactive_tool_button);

   coords_layout.addWidget(coords_widgets.at(0),0,0);
   coords_layout.addWidget(coords_widgets.at(1),0,1);
   coords_layout.addWidget(coords_widgets.at(2),0,2);
   coords_layout.addWidget(coords_widgets.at(3),1,0);
   coords_layout.addWidget(coords_widgets.at(4),1,1);
   coords_layout.addWidget(coords_widgets.at(5),1,2);
   
   main_layout.addLayout(&coords_layout);
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

void XBot::widgets::im_widget::im_callback(const visualization_msgs::InteractiveMarkerFeedback& feedback)
{
    marker.pose = feedback.pose;

    changing_test.store(true);
    update_text();
    changing_test.store(false);
}

void XBot::widgets::im_widget::update_text()
{
    coords_widgets.at(0)->edit.setText(QString::number(marker.pose.position.x, 'f', 3));
    coords_widgets.at(1)->edit.setText(QString::number(marker.pose.position.y, 'f', 3));
    coords_widgets.at(2)->edit.setText(QString::number(marker.pose.position.z, 'f', 3));
    double ro,pi,ya;
    tf::Quaternion q;
    tf::quaternionMsgToTF(marker.pose.orientation,q);
    tf::Matrix3x3(q).getRPY(ro, pi, ya);
    coords_widgets.at(3)->edit.setText(QString::number(ro, 'f', 3));
    coords_widgets.at(4)->edit.setText(QString::number(pi, 'f', 3));
    coords_widgets.at(5)->edit.setText(QString::number(ya, 'f', 3));
}

void XBot::widgets::im_widget::on_text_edit_changed(int id)
{
    if(!changing_test.load())
    {
	marker.pose.position.x = coords_widgets.at(0)->edit.text().toDouble();
	marker.pose.position.y = coords_widgets.at(1)->edit.text().toDouble();
	marker.pose.position.z = coords_widgets.at(2)->edit.text().toDouble();
	marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(coords_widgets.at(3)->edit.text().toDouble(),coords_widgets.at(4)->edit.text().toDouble(),coords_widgets.at(5)->edit.text().toDouble());

	on_publish_button_clicked();
    }
}

XBot::widgets::im_widget::~im_widget()
{

}