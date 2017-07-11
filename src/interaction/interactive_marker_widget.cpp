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

XBot::widgets::im_widget::im_widget(rviz::ToolManager* tool_manager_, std::string name_, QComboBox& object_combo_)
: QWidget(), tool_manager(tool_manager_), name(name_), object_combo(object_combo_)
{
   changing_coords.store(false);
   changing_scale.store(false);

   pose_service = nh.advertiseService(name+"_pose",&im_widget::pose_service_callback,this);
   pose_array_service = nh.advertiseService(name+"_pose_array",&im_widget::pose_array_service_callback,this);
   marker_pub = nh.advertise<visualization_msgs::Marker>(name+"_client",1);
   publish_button.setText("Publish Marker");
   im_sub = nh.subscribe(("/"+name+"_server/feedback").c_str(),1,&im_widget::im_callback,this);

   position_by_click_button.setCheckable(true);
   position_by_click_button.setText("Position by Click");
   click_tool = tool_manager->addTool("rviz/PublishPoint");
   click_tool->getPropertyContainer()->subProp("Topic")->setValue(("/" +name+ "_clicked_point").c_str());
   position_by_click_sub = nh.subscribe("/" +name+ "_clicked_point",1,&im_widget::position_by_click_callback,this);

   coords_widgets[0] = new label_lineedit("x:");
   coords_widgets[1] = new label_lineedit("y:");
   coords_widgets[2] = new label_lineedit("z:");
   coords_widgets[3] = new label_lineedit("R:");
   coords_widgets[4] = new label_lineedit("P:");
   coords_widgets[5] = new label_lineedit("Y:");

   for(int i=0;i<coords_widgets.size();i++)
   {
      connect(&(coords_widgets.at(i)->edit), SIGNAL(textChanged(QString)),&coord_mapper, SLOT(map()));
      coord_mapper.setMapping(&(coords_widgets.at(i)->edit), i);
   }
   connect(&coord_mapper, SIGNAL(mapped(int)), this, SLOT(on_coords_changed(int))) ;

   scale_label.setText("Scale   ");
   scale_widgets[0] = new label_lineedit("x:");
   scale_widgets[1] = new label_lineedit("y:");
   scale_widgets[2] = new label_lineedit("z:");

   buttons_layout.addWidget(&publish_button);
   buttons_layout.addWidget(&position_by_click_button);

   coords_layout.addWidget(coords_widgets.at(0),0,0);
   coords_layout.addWidget(coords_widgets.at(1),0,1);
   coords_layout.addWidget(coords_widgets.at(2),0,2);
   coords_layout.addWidget(coords_widgets.at(3),1,0);
   coords_layout.addWidget(coords_widgets.at(4),1,1);
   coords_layout.addWidget(coords_widgets.at(5),1,2);
   
   scale_layout.addWidget(&scale_label);
   scale_layout.addWidget(scale_widgets.at(0));
   scale_layout.addWidget(scale_widgets.at(1));
   scale_layout.addWidget(scale_widgets.at(2));
   
   main_layout.addLayout(&coords_layout);
   main_layout.addLayout(&scale_layout);
   main_layout.addLayout(&buttons_layout);
   
   setLayout(&main_layout);
   
   connect(&publish_button, SIGNAL(clicked(bool)), this, SLOT(on_publish_button_clicked()));
   connect(&position_by_click_button, SIGNAL(clicked(bool)), this, SLOT(on_position_by_click_button_clicked()));
   
   marker.color.g=1;
   marker.color.a=1;
   marker.pose.orientation.w=1;
   marker.header.frame_id="/base_link";
   
   for(int i=0;i<3;i++)
   {
      connect(&(scale_widgets.at(i)->edit), SIGNAL(textChanged(QString)),&scale_mapper, SLOT(map()));
      scale_mapper.setMapping(&(scale_widgets.at(i)->edit), i);
   }
   connect(&scale_mapper, SIGNAL(mapped(int)), this, SLOT(on_scale_changed(int))) ;
}

bool XBot::widgets::im_widget::pose_service_callback(ADVR_ROS::im_pose::Request& req, ADVR_ROS::im_pose::Response& res)
{
    res.pose_stamped.header = marker.header;
    res.pose_stamped.pose = marker.pose;

    return true;
}

bool XBot::widgets::im_widget::pose_array_service_callback(ADVR_ROS::im_pose_array::Request& req, ADVR_ROS::im_pose_array::Response& res)
{
    for(auto object:objects)
    {
	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.header = marker.header;
	pose_stamped.pose.position.x = object.second.pose.position.x;
	pose_stamped.pose.position.y = object.second.pose.position.y;
	pose_stamped.pose.position.z = object.second.pose.position.z;
	pose_stamped.pose.orientation.x = object.second.pose.orientation.x;
	pose_stamped.pose.orientation.y = object.second.pose.orientation.y;
	pose_stamped.pose.orientation.z = object.second.pose.orientation.z;
	pose_stamped.pose.orientation.w = object.second.pose.orientation.w;
	res.poses_stamped.push_back(pose_stamped);
    }

    return true;
}

void XBot::widgets::im_widget::position_by_click_callback(const geometry_msgs::PointStamped& point)
{
    //resetting button
    position_by_click_button.setChecked(false);
    position_by_click_button.setText("Position by Click");
    tool_manager->setCurrentTool(last_tool);

    //saving data
    marker.pose.position.x = point.point.x;
    marker.pose.position.y = point.point.y;
    marker.pose.position.z = point.point.z;

    changing_coords.store(true);
    update_coords();
    changing_coords.store(false);

    on_publish_button_clicked();
}

void XBot::widgets::im_widget::on_position_by_click_button_clicked()
{
    if(position_by_click_button.isChecked())
    {
        last_tool = tool_manager->getCurrentTool();
	tool_manager->setCurrentTool(click_tool);
	position_by_click_button.setText("ABORT Positioning");
    }
    else
    {
	tool_manager->setCurrentTool(last_tool);
	position_by_click_button.setText("Position by Click");
    }
}

void XBot::widgets::im_widget::on_publish_button_clicked()
{
    marker_pub.publish(marker);
}

void XBot::widgets::im_widget::im_callback(const visualization_msgs::InteractiveMarkerFeedback& feedback)
{
    int id = std::stoi(feedback.marker_name);
    if(id!=marker.id)
        if(combo_ids.count(id))
	    object_combo.setCurrentIndex(combo_ids.at(id));

    marker.pose = feedback.pose;
    
    objects.at(object_combo.currentText().toStdString()).pose = feedback.pose;

    update_coords();
    on_publish_button_clicked();
}

void XBot::widgets::im_widget::update_coords()
{
    changing_coords.store(true);

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

    changing_coords.store(false);
}

void XBot::widgets::im_widget::on_coords_changed(int id)
{
    if(!changing_coords.load())
    {
        objects.at(object_combo.currentText().toStdString()).pose.position.x = coords_widgets.at(0)->edit.text().toDouble();
	objects.at(object_combo.currentText().toStdString()).pose.position.y = coords_widgets.at(1)->edit.text().toDouble();
	objects.at(object_combo.currentText().toStdString()).pose.position.z = coords_widgets.at(2)->edit.text().toDouble();
	objects.at(object_combo.currentText().toStdString()).pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(coords_widgets.at(3)->edit.text().toDouble(),coords_widgets.at(4)->edit.text().toDouble(),coords_widgets.at(5)->edit.text().toDouble());

	marker.pose.position.x = coords_widgets.at(0)->edit.text().toDouble();
	marker.pose.position.y = coords_widgets.at(1)->edit.text().toDouble();
	marker.pose.position.z = coords_widgets.at(2)->edit.text().toDouble();
	marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(coords_widgets.at(3)->edit.text().toDouble(),coords_widgets.at(4)->edit.text().toDouble(),coords_widgets.at(5)->edit.text().toDouble());

	on_publish_button_clicked();
    }
}

void XBot::widgets::im_widget::on_scale_changed(int id)
{
    if(!changing_scale.load())
    {
	objects.at(object_combo.currentText().toStdString()).scale.x = scale_widgets.at(0)->edit.text().toDouble();
	objects.at(object_combo.currentText().toStdString()).scale.y = scale_widgets.at(1)->edit.text().toDouble();
	objects.at(object_combo.currentText().toStdString()).scale.z = scale_widgets.at(2)->edit.text().toDouble();
	
	marker.scale.x = scale_widgets.at(0)->edit.text().toDouble();
	marker.scale.y = scale_widgets.at(1)->edit.text().toDouble();
	marker.scale.z = scale_widgets.at(2)->edit.text().toDouble();
    }
    
    on_publish_button_clicked();
}

void XBot::widgets::im_widget::update_scale()
{
    changing_scale.store(true);
    scale_widgets.at(0)->edit.setText(QString::number(objects.at(object_combo.currentText().toStdString()).scale.x, 'f', 3));
    scale_widgets.at(1)->edit.setText(QString::number(objects.at(object_combo.currentText().toStdString()).scale.y, 'f', 3));
    scale_widgets.at(2)->edit.setText(QString::number(objects.at(object_combo.currentText().toStdString()).scale.z, 'f', 3));
    changing_scale.store(false);
}

void XBot::widgets::im_widget::load_object_params()
{
    marker.id = objects.at(object_combo.currentText().toStdString()).id;
    marker.type = objects.at(object_combo.currentText().toStdString()).type;
    marker.mesh_resource = objects.at(object_combo.currentText().toStdString()).mesh_name;
    marker.scale.x = objects.at(object_combo.currentText().toStdString()).scale.x;
    marker.scale.y = objects.at(object_combo.currentText().toStdString()).scale.y;
    marker.scale.z = objects.at(object_combo.currentText().toStdString()).scale.z;
    marker.pose.position.x = objects.at(object_combo.currentText().toStdString()).pose.position.x;
    marker.pose.position.y = objects.at(object_combo.currentText().toStdString()).pose.position.y;
    marker.pose.position.z = objects.at(object_combo.currentText().toStdString()).pose.position.z;
    marker.pose.orientation.x = objects.at(object_combo.currentText().toStdString()).pose.orientation.x;
    marker.pose.orientation.y = objects.at(object_combo.currentText().toStdString()).pose.orientation.y;
    marker.pose.orientation.z = objects.at(object_combo.currentText().toStdString()).pose.orientation.z;
    marker.pose.orientation.w = objects.at(object_combo.currentText().toStdString()).pose.orientation.w;
}

void XBot::widgets::im_widget::object_combo_changed()
{
    if(object_combo.count()==0) return;

    load_object_params();
    update_coords();
    on_coords_changed(0);
    update_scale();
    on_scale_changed(0);

    marker.action = visualization_msgs::Marker::ADD;
    on_publish_button_clicked();
}

void XBot::widgets::im_widget::generate_objects(std::map< std::string, XBot::object_properties > objects_)
{
    std::map<int,bool> ids;
    int i=0;

    for(auto object_:objects_)
    {
	objects[object_.first] = object_.second;
	object_combo.addItem(QString::fromStdString(object_.first));

	ids[objects.at(object_.first).id] = true;
	combo_ids[object_.second.id]=i;
	i++;
	objects.at(object_.first).mesh_name = "file://"+ std::string(getenv("ROBOTOLOGY_ROOT")) +"/external/XBotGUI/resources/" + object_.second.mesh_name;
    }

    object_combo.setCurrentIndex(0);
    load_object_params();
    update_scale();
}

void XBot::widgets::im_widget::add_object(XBot::object_properties object_)
{
    std::string new_name = std::to_string(objects.size());
    objects[new_name] = object_;
    objects.at(new_name).mesh_name = "file://"+ std::string(getenv("ROBOTOLOGY_ROOT")) +"/external/XBotGUI/resources/" + object_.mesh_name;
}

void XBot::widgets::im_widget::delete_last_object()
{
    if(objects.size()>0)
    {
	marker.action = visualization_msgs::Marker::DELETE;
	marker.id = objects.size()-1;
	on_publish_button_clicked();
	objects.erase(std::to_string(objects.size()-1));
    }
}

XBot::widgets::im_widget::~im_widget()
{
    for(auto coord:coords_widgets)
	delete coord.second;
    for(auto scale:scale_widgets)
	delete scale.second;
}