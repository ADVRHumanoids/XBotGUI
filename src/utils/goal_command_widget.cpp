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

#include <XBotGUI/utils/goal_command_widget.h>

XBot::widgets::goal_command_widget::goal_command_widget(rviz::ToolManager* tool_manager_, std::string topic_name_):
tool_manager(tool_manager_), topic_name(topic_name_)
{
    pub = nh.advertise<geometry_msgs::PoseStamped>(topic_name.c_str(),1);

    std::string tool_topic = topic_name + "_SetGoal";
    sub = nh.subscribe(tool_topic.c_str(),1,&goal_command_widget::goal_callback,this);

    goal_tool = tool_manager->addTool("rviz/SetGoal");
    goal_tool->getPropertyContainer()->subProp("Topic")->setValue(tool_topic.c_str());

    select_goal_button.setCheckable(true);
    select_goal_button.setText("Select Goal");
    show_goal_button.setText("Show Goal");
    send_goal_button.setText("Send Goal");

    coords_widgets[0] = new label_lineedit("x:");
    coords_widgets[1] = new label_lineedit("y:");
    coords_widgets[2] = new label_lineedit("Y:");

    for(int i=0;i<coords_widgets.size();i++)
    {
	connect(&(coords_widgets.at(i)->edit), SIGNAL(textChanged(QString)),&coord_mapper, SLOT(map()));
	coord_mapper.setMapping(&(coords_widgets.at(i)->edit), i);
	coords_layout.addWidget(coords_widgets.at(i),0,i);
    }
    connect(&coord_mapper, SIGNAL(mapped(int)), this, SLOT(on_coords_changed(int))) ;

    c_layout.addWidget(&select_goal_button);
    c_layout.addWidget(&show_goal_button);
    main_layout.addLayout(&c_layout);
    main_layout.addLayout(&coords_layout);
    main_layout.addWidget(&send_goal_button);

    connect(&select_goal_button,SIGNAL(clicked()),this,SLOT(on_select_goal_button_clicked()));
    connect(&show_goal_button,SIGNAL(clicked()),this,SLOT(on_show_goal_button_clicked()));
    connect(&send_goal_button,SIGNAL(clicked()),this,SLOT(on_send_goal_button_clicked()));

    last_pose.pose.orientation.w=1;

    setLayout(&main_layout);

    marker.color.b=1;
    marker.color.a=1;
    marker.pose.orientation.w=1;
    marker.header.frame_id="/base_link";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.5;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    marker_pub = nh.advertise<visualization_msgs::Marker>(topic_name_+"_goal_marker",1);
}

void XBot::widgets::goal_command_widget::set_fixed_frame(std::string frame)
{
    std::string err_msg;
    if(tf_.waitForTransform(marker.header.frame_id,frame,ros::Time::now(), ros::Duration(1.0), ros::Duration(0.01), &err_msg))
    {
        geometry_msgs::PoseStamped input;
	input.header.frame_id = marker.header.frame_id;
	input.pose = marker.pose;
	geometry_msgs::PoseStamped output;
	tf_.transformPose(frame,input,output);
	marker.pose = output.pose;
	marker.header.frame_id=frame;
	last_pose.pose = output.pose;
	last_pose.header.frame_id=frame;
	
	update_coords(output.pose);
	update_marker();
    }
    else
    {
	std::cout<<red_string("ERROR: TF not found between " + marker.header.frame_id + " and " + frame)<<std::endl;
    }
}

void XBot::widgets::goal_command_widget::update_coords(const geometry_msgs::Pose& pose)
{
    changing_coords.store(true);
    coords_widgets.at(0)->edit.setText(QString::number(pose.position.x, 'f', 3));
    coords_widgets.at(1)->edit.setText(QString::number(pose.position.y, 'f', 3));
    double ro,pi,ya;
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose.orientation,q);
    tf::Matrix3x3(q).getRPY(ro, pi, ya);
    coords_widgets.at(2)->edit.setText(QString::number(ya, 'f', 3));
    changing_coords.store(false);
}

void XBot::widgets::goal_command_widget::update_marker()
{
    marker.pose.position.x = coords_widgets.at(0)->edit.text().toDouble();
    marker.pose.position.y = coords_widgets.at(1)->edit.text().toDouble();
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,coords_widgets.at(2)->edit.text().toDouble());
}

void XBot::widgets::goal_command_widget::publish_marker()
{
    marker_pub.publish(marker);
}

void XBot::widgets::goal_command_widget::on_coords_changed(int id)
{
    if(!changing_coords.load())
    {
        last_pose.pose.position.x = coords_widgets.at(0)->edit.text().toDouble();
	last_pose.pose.position.y = coords_widgets.at(1)->edit.text().toDouble();
	last_pose.pose.position.z = 0.0;
	last_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,coords_widgets.at(2)->edit.text().toDouble());

	update_marker();
	publish_marker();
    }
}

void XBot::widgets::goal_command_widget::goal_callback(const geometry_msgs::PoseStamped& pose)
{
    last_pose = pose;

    update_coords(last_pose.pose);

    update_marker();
    publish_marker();

    tool_manager->setCurrentTool(last_tool);
    select_goal_button.setText("Select Goal");
    select_goal_button.setChecked(false);
}

void XBot::widgets::goal_command_widget::on_select_goal_button_clicked()
{
    if(select_goal_button.isChecked())
    {
        last_tool = tool_manager->getCurrentTool();
	tool_manager->setCurrentTool(goal_tool);
	select_goal_button.setText("ABORT Goal Selection");
    }
    else
    {
	tool_manager->setCurrentTool(last_tool);
	select_goal_button.setText("Select Goal");
    }
}

void XBot::widgets::goal_command_widget::on_show_goal_button_clicked()
{
    publish_marker();
}

void XBot::widgets::goal_command_widget::on_send_goal_button_clicked()
{
    pub.publish(last_pose);
}