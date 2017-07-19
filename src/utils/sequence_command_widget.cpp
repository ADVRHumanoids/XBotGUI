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

#include <XBotGUI/utils/sequence_command_widget.h>

XBot::widgets::sequence_command_widget::sequence_command_widget(std::string topic_name_, std::string service_name_): topic_name(topic_name_), service_name(service_name_)
{
    sequence_client = nh.serviceClient<ADVR_ROS::im_pose_array>((service_name+"_pose_array").c_str());
    pub = nh.advertise<ADVR_ROS::im_pose_array_msg>(topic_name.c_str(),1);

    sequence_button.setText(QString::fromStdString("Send Object Sequence"));
    
    main_layout.addWidget(&sequence_button);

    connect(&sequence_button,SIGNAL(clicked()),this,SLOT(on_sequence_button_clicked()));

    setLayout(&main_layout);

    thread_waiting.store(false);
}

void XBot::widgets::sequence_command_widget::service_thread_body()
{
    ADVR_ROS::im_pose_array srv;
    srv.request.name = topic_name;
    if(sequence_client.call(srv))
    {
        ADVR_ROS::im_pose_array_msg poses;
	poses.name = topic_name;
	for(auto ps:srv.response.im_poses_array.im_poses)
	{
	    poses.im_poses.push_back(ps);
	}

        pub.publish(poses);
    }
    else
    {
	ROS_ERROR_STREAM("Error calling "<<service_name<<" service");
    }
    thread_waiting.store(false);
}

void XBot::widgets::sequence_command_widget::on_sequence_button_clicked()
{
    std::thread service_thread(&sequence_command_widget::service_thread_body,this);
    thread_waiting.store(true);

    while(thread_waiting.load())
    {
	ros::spinOnce();
    }

    if(service_thread.joinable()) service_thread.join();
    
}