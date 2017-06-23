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

#include <XBotGUI/utils/pose_command_widget.h>

pose_command_widget::pose_command_widget(std::string topic_name_, std::string service_name_): QWidget(), topic_name(topic_name_), service_name(service_name_)
{
    pose_client = nh.serviceClient<ADVR_ROS::im_pose>((service_name+"_pose").c_str());
    pub = nh.advertise<geometry_msgs::PoseStamped>(topic_name.c_str(),1);

    pose_button.setText(QString::fromStdString("Send Object Pose"));
    
    main_layout.addWidget(&pose_button);

    connect(&pose_button,SIGNAL(clicked()),this,SLOT(on_pose_button_clicked()));

    setLayout(&main_layout);

    thread_waiting.store(false);
}

void pose_command_widget::service_thread_body()
{
    ADVR_ROS::im_pose srv;
    srv.request.name = topic_name;
    if(pose_client.call(srv))
    {
        pub.publish(srv.response.pose_stamped);
    }
    else
    {
	ROS_ERROR_STREAM("Error calling "<<service_name<<" service");
    }
    thread_waiting.store(false);
}

void pose_command_widget::on_pose_button_clicked()
{
    std::thread service_thread(&pose_command_widget::service_thread_body,this);
    thread_waiting.store(true);

    while(thread_waiting.load())
    {
	ros::spinOnce();
    }

    if(service_thread.joinable()) service_thread.join();
    
}