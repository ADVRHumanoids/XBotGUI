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

#include <XBotGUI/utils/cmd_service_widget.h>

XBot::widgets::cmd_service_widget::cmd_service_widget(std::string module_name_, std::string command_name_): module_name(module_name_), command_name(command_name_)
{
    cmd_client = nh.serviceClient<XCM::cmd_service>((module_name+"_cmd").c_str());

    cmd_button.setText(QString::fromStdString(command_name));
    
    main_layout.addWidget(&cmd_button);

    connect(&cmd_button,SIGNAL(clicked()),this,SLOT(on_cmd_button_clicked()));

    setLayout(&main_layout);

    thread_waiting.store(false);
}

void XBot::widgets::cmd_service_widget::set_label(std::string label_name)
{
    cmd_button.setText(QString::fromStdString(label_name));
}

void XBot::widgets::cmd_service_widget::service_thread_body()
{
    XCM::cmd_service srv;
    srv.request.cmd = command_name;
    if(cmd_client.call(srv))
    {
        if(!srv.response.success) ROS_ERROR_STREAM("Service "<<module_name<<"_cmd returned FAILURE");
    }
    else
    {
	ROS_ERROR_STREAM("Error calling "<<module_name<<"_cmd service");
    }
    thread_waiting.store(false);
}

void XBot::widgets::cmd_service_widget::on_cmd_button_clicked()
{
    std::thread service_thread(&cmd_service_widget::service_thread_body,this);
    thread_waiting.store(true);

    while(thread_waiting.load())
    {
	ros::spinOnce();
    }

    if(service_thread.joinable()) service_thread.join();
    
}