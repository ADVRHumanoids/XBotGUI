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

#include "XBotGUI/utils/status_widget.h"
#include "XBotGUI/interaction/module.h"

XBot::widgets::status_widget::status_widget(XBot::widgets::module* top_module_, std::string module_name): top_module(top_module_)
{
    sub = nh.subscribe((module_name+"_status_aux").c_str(),1,&status_widget::status_callback,this);

    status_timer.setInterval(2000);

    status_edit.setText("-");
    main_layout.addWidget(&status_edit);

    connect(&status_timer, SIGNAL(timeout()), this, SLOT(status_timer_body())); 

    setLayout(&main_layout);
}

void XBot::widgets::status_widget::status_callback(const std_msgs::String& status)
{
    status_edit.setText(QString::fromStdString(status.data));

    top_module->status_changed(status.data);

    status_timer.start(2000);
}

void XBot::widgets::status_widget::status_timer_body()
{
    status_edit.setText("-");
}