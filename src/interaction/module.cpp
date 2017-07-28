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

void XBot::widgets::module::start_info(bool error, std::string plugin_name, bool called)
{
    if(!error)
    {
	std::cout<<green_string("[ " + plugin_name +" ") + yellow_string("STARTED") + green_string(" ]")<<std::endl;
    }
    else
    {
	if(!called)
	    std::cout<<red_string("[ ERROR CALLING START SERVICE FOR " + plugin_name +" ]")<<std::endl;
	else
	    std::cout<<red_string("[ ERROR STARTING " + plugin_name +" ]")<<std::endl;
    }
}

void XBot::widgets::module::stop_info(bool error, std::string plugin_name, bool called)
{
    if(!error)
    {
	std::cout<<green_string("[ " + plugin_name +" ") + purple_string("STOPPED") + green_string(" ]")<<std::endl;
    }
    else
    {
        if(!called)
	    std::cout<<red_string("[ ERROR CALLING STOP SERVICE FOR " + plugin_name +" ]")<<std::endl;
	else
	    std::cout<<red_string("[ ERROR STOPPING " + plugin_name +" ]")<<std::endl;
    }
}

XBot::widgets::module::module(std::string name_, std::vector<std::vector<std::map<std::string,std::string>>> command_blocks_, std::vector<std::string> module_dependencies, rviz::ToolManager* tool_manager_)
: QWidget(), name(name_), status_wid(this,name_)
{
    for(auto dep:module_dependencies)
    {
        switch_client.push_back(nh.serviceClient<std_srvs::SetBool>((dep+"_switch").c_str()));
    }

    switch_client.push_back(nh.serviceClient<std_srvs::SetBool>((name+"_switch").c_str()));

    switch_button.setCheckable(true);
    switch_button.setText("Start");

    basic_layout.addWidget(&switch_button);
    basic_layout.addWidget(&status_wid);
    
    main_layout.addLayout(&basic_layout);

    for(auto command_block:command_blocks_)
    {
        QHBoxLayout* l = new QHBoxLayout();

	for(auto command:command_block)
	{
	    if(command.at("type")=="object_pose")
	    {
		command_widgets.push_back(new pose_command_widget(command.at("topic"),command.at("interactive_marker")));
	    }
	    else if(command.at("type")=="goal")
	    {
		command_widgets.push_back(new goal_command_widget(tool_manager_,command.at("topic")));
	    }
	    else if(command.at("type")=="object_sequence")
	    {
		command_widgets.push_back(new sequence_command_widget(command.at("topic"),command.at("interactive_markers_sequence")));
	    }
	    else if(command.at("type")=="cmd_service")
	    {
	        command_widgets.push_back(new cmd_service_widget(name,command.at("name")));

	        if(command.count("label"))
		{
		    ((cmd_service_widget*)(command_widgets.back()))->set_label(command.at("label"));
		}
	    }
	    else if(command.at("type")=="std_srvs/Empty")
	    {
		command_widgets.push_back(new empty_service_widget(command.at("service_name"),command.at("name")));
	    }
	    else if(command.at("type")=="grasping")
	    {
		command_widgets.push_back(new grasp_widget());
	    }
	    else if(command.at("type")=="string")
	    {
		command_widgets.push_back(new string_command_widget(command.at("topic"),command.at("name")));
	    }
	    else continue;

	    if(command.count("enabling_status"))
	    {
		command_widgets.back()->set_enabling_status(command.at("enabling_status"));
	    }

	    l->addWidget(command_widgets.back());
	}

	h_layout.push_back(l);
    }

    for(auto l:h_layout)
    {
	main_layout.addLayout(l);
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

void XBot::widgets::module::status_changed(std::string name)
{
    for(auto cmd_w:command_widgets)
    {
	if(cmd_w->get_enabling_status()!="")
	      cmd_w->setEnabled(((cmd_service_widget*)(cmd_w))->get_enabling_status()==name);
    }
}

void XBot::widgets::module::on_switch_button_clicked()
{
    if(switch_button.isChecked())
    {
	switch_button.setText("Stop");
	switch_service.request.data = true;

	for(auto& sc:switch_client)
	{
            sleep(1);
	    if(sc.call(switch_service))
	    {
		if(switch_service.response.success)
		    start_info(false,sc.getService(),true);
		else
		    start_info(true,sc.getService(),true);
	    }
	    else
	    {
		start_info(true,sc.getService(),false);
	    }
	}
    }
    else
    {
	switch_button.setText("Start");
	switch_service.request.data = false;
	
	for(auto& sc:switch_client)
	{
        sleep(1);
	    if(sc.call(switch_service))
	    {
		if(switch_service.response.success)
		    stop_info(false,sc.getService(),true);
		else
		    stop_info(true,sc.getService(),true);
	    }
	    else
	    {
		stop_info(true,sc.getService(),false);
	    }
	}
    }
}

XBot::widgets::module::~module()
{
    for(auto cw:command_widgets)
    {
	delete cw;
    }
    for(auto l:h_layout)
    {
	delete l;
    }
}