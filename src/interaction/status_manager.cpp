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

#include "XBotGUI/interaction/status_manager.h"

XBot::status_manager::status_manager()
{
    shutdown.store(false);
    status_thread =  new std::thread(&status_manager::status_thread_body,this);
}

void XBot::status_manager::add_module_status_service(std::string module_name)
{
    status_clients.push_back(nh.serviceClient<XCM::status_service>((module_name+"_status").c_str()));
    status_pubs.push_back(nh.advertise<std_msgs::String>((module_name+"_status_aux").c_str(),1));
}

void XBot::status_manager::status_thread_body()
{
    ros::Rate loop(10);

    while(ros::ok() && !shutdown.load())
    {
	for(int i=0; i<status_clients.size(); i++)
	{
	    XCM::status_service srv;
	    std_msgs::String status;
	    if(status_clients.at(i).call(srv))
	    {
	        status.data = srv.response.status;
	        status_pubs.at(i).publish(status);
	    }
	    else
	    {
		status.data = "NOT ACTIVE";
	        status_pubs.at(i).publish(status);
	    }
	}

	loop.sleep();
    }
}

XBot::status_manager::~status_manager()
{
    shutdown.store(true);
    if(status_thread->joinable()) status_thread->join();
    delete status_thread;
}