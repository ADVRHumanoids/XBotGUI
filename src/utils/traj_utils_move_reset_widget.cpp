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

#include <XBotGUI/utils/traj_utils_move_reset_widget.h>

XBot::widgets::traj_utils_move_reset_widget::traj_utils_move_reset_widget(std::string marker_name_, std::string command_name_):
marker_name(marker_name_), command_name(command_name_)
{
    feedback_msg.header.frame_id = "world_odom";
    feedback_msg.client_id = "/XBotGUI/";
    feedback_msg.marker_name = marker_name;
    feedback_msg.event_type = 2;
    feedback_msg.pose.orientation.w=1;
    feedback_msg.menu_entry_id = 41;
    feedback_msg.mouse_point_valid=1;
    
    service_name = marker_name + "_getTrj";
    feedback_name = marker_name + "_trajectory_marker_server/feedback";
    
    feedback_publisher = nh.advertise<visualization_msgs::InteractiveMarkerFeedback>(feedback_name.c_str(),1);

    empty_client = nh.serviceClient<std_srvs::Empty>((service_name).c_str());

    cmd_button.setText(QString::fromStdString(command_name));
    
    main_layout.addWidget(&cmd_button);

    connect(&cmd_button,SIGNAL(clicked()),this,SLOT(on_cmd_button_clicked()));

    setLayout(&main_layout);

    thread_waiting.store(false);
}

void XBot::widgets::traj_utils_move_reset_widget::service_thread_body()
{
    std_srvs::Empty srv;
    if(empty_client.call(srv))
    {
	feedback_publisher.publish(feedback_msg);
    }
    else
    {
	ROS_ERROR_STREAM("Error calling "<<service_name<<" service");
    }
    thread_waiting.store(false);
}

void XBot::widgets::traj_utils_move_reset_widget::on_cmd_button_clicked()
{
    std::thread service_thread(&traj_utils_move_reset_widget::service_thread_body,this);
    thread_waiting.store(true);

    while(thread_waiting.load())
    {
	ros::spinOnce();
    }

    if(service_thread.joinable()) service_thread.join();
    
}