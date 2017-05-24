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

#ifndef XBOTGUI_MODULE_H
#define XBOTGUI_MODULE_H

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <QBoxLayout>
#include <QPushButton>

namespace XBot
{
namespace widgets
{
class module: public QWidget
{
Q_OBJECT
public:
    module(std::string name_);
    ~module();

private Q_SLOTS:
    void on_switch_button_clicked();

private:
    void start_info(bool error);
    void stop_info(bool error);

    std::string name;
    ros::NodeHandle nh;
    ros::ServiceClient switch_client;
    std_srvs::SetBool switch_service;

    QPushButton switch_button;

    QVBoxLayout main_layout;

};
};
};

#endif