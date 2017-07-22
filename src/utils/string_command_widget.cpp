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

#include <XBotGUI/utils/string_command_widget.h>

XBot::widgets::string_command_widget::string_command_widget(std::string topic_name_, std::string command_name_):
topic_name(topic_name_), command_name(command_name_)
{
    pub = nh.advertise<std_msgs::String>(topic_name.c_str(),1);

    send_command_button.setText(QString::fromStdString(command_name));

    main_layout.addWidget(&send_command_button);

    connect(&send_command_button,SIGNAL(clicked()),this,SLOT(on_send_command_button_clicked()));
}

void XBot::widgets::string_command_widget::on_send_command_button_clicked()
{
    std_msgs::String cmd;
    cmd.data=command_name;
    pub.publish(cmd);
}