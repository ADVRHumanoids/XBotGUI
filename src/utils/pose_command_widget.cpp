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

pose_command_widget::pose_command_widget(std::string name_, std::string type_): QWidget(), name(name_), type(type_)
{
    pose_client = nh.serviceClient<ADVR_ROS::im_pose>(type.c_str());
    pub = nh.advertise<geometry_msgs::PoseStamped>(name.c_str(),1);

    pose_button.setText(QString::fromStdString(name));
    
    main_layout.addWidget(&pose_button);

    connect(&pose_button,SIGNAL(clicked()),this,SLOT(on_pose_button_clicked()));

    setLayout(&main_layout);

    thread_waiting.store(false);
}

void pose_command_widget::service_thread_body()
{
    ADVR_ROS::im_pose srv;
    srv.request.name = name;
    if(pose_client.call(srv))
    {
        pub.publish(srv.response.pose_stamped);
    }
    else
    {
	ROS_ERROR_STREAM("Error calling "<<type<<" service");
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