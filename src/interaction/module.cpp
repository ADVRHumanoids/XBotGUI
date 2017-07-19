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
#include "XBotGUI/interaction/module.h"
#include "XBotGUI/print_utils.h"

void XBot::widgets::module::start_info(bool error)
{
    if(!error)
    {
	std::cout<<green_string("[ MODULE " + name +" ") + yellow_string("STARTED") + green_string(" ]")<<std::endl;
    }
    else
    {
	std::cout<<red_string("[ ERROR STARTING MODULE " + name +" ]")<<std::endl;
    }
}

void XBot::widgets::module::stop_info(bool error)
{
    if(!error)
    {
	std::cout<<green_string("[ MODULE " + name +" ") + yellow_string("STOPPED") + green_string(" ]")<<std::endl;
    }
    else
    {
	std::cout<<red_string("[ ERROR STOPPING MODULE " + name +" ]")<<std::endl;
    }
}

XBot::widgets::module::module(std::string name_, std::vector<std::map<std::string,std::string>> commands_, rviz::ToolManager* tool_manager_): QWidget(), name(name_)
{
    switch_client = nh.serviceClient<std_srvs::SetBool>((name+"_switch").c_str());

    switch_button.setCheckable(true);
    switch_button.setText("Start");

    main_layout.addWidget(&switch_button);

    for(auto command:commands_)
    {
	if(command.at("type")=="object_pose")
	{
	    command_widgets.push_back(new pose_command_widget(command.at("topic"),command.at("interactive_marker")));
	}

	if(command.at("type")=="goal")
	{
	    command_widgets.push_back(new goal_command_widget(tool_manager_,command.at("topic")));
	}

	if(command.at("type")=="object_sequence")
	{
	    command_widgets.push_back(new sequence_command_widget(command.at("topic"),command.at("interactive_markers_sequence")));
	}

	if(command.at("type")=="cmd_service")
	{
	    command_widgets.push_back(new string_command_widget(name,command.at("name")));
	}

	main_layout.addWidget(command_widgets.back());
    }

    connect(&switch_button,SIGNAL(clicked()),this,SLOT(on_switch_button_clicked()));

    setLayout(&main_layout);
}

QPushButton* XBot::widgets::module::get_switch_button()
{
    return &switch_button;
}

void XBot::widgets::module::set_fixed_frame(std::string frame)
{
    for(auto cmd_w:command_widgets)
    {
	cmd_w->set_fixed_frame(frame);
    }
}

void XBot::widgets::module::on_switch_button_clicked()
{
    if(switch_button.isChecked())
    {
	switch_button.setText("Stop");
	switch_service.request.data = true;
	if(switch_client.call(switch_service))
	{
	    if(switch_service.response.success)
		start_info(false);
	    else
		start_info(true);
	}
	else
	{
	    start_info(true);
	}
    }
    else
    {
	switch_button.setText("Start");

	switch_service.request.data = false;
	if(switch_client.call(switch_service))
	{
	    if(switch_service.response.success)
		stop_info(false);
	    else
		stop_info(true);
	}
	else
	{
	    stop_info(true);
	}
    }
}

XBot::widgets::module::~module()
{
    for(auto cw:command_widgets)
    {
	delete cw;
    }
}