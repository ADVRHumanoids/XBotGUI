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

#ifndef XBOTGUI_STATUS_MANAGER_H
#define XBOTGUI_STATUS_MANAGER_H

#include <string>
#include <vector>
#include <thread>
#include <ros/ros.h>
#include <ros/service.h>
#include <std_msgs/String.h>
#include <XCM/status_service.h>

namespace XBot
{
class status_manager
{
public:
    status_manager();
    ~status_manager();

    void add_module_status_service(std::string module_name);

private:
    ros::NodeHandle nh;

    std::vector<ros::ServiceClient> status_clients;
    std::vector<ros::Publisher> status_pubs;

    std::thread* status_thread;
    void status_thread_body();

};
};

#endif