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
#include "XBotGUI/main_interface.h"
#include <iostream>
#include <string>
#include <cstdlib>

XBot::GUI::GUI(std::string config_file): QWidget()
{
    std::string filename = "/build/external/XBotGUI/configs/xbotgui.xml";
    filename = std::getenv("ROBOTOLOGY_ROOT") + filename;

    TiXmlDocument doc(filename);
    bool loadOkay = doc.LoadFile();
    if (!loadOkay)
    {
        std::cout<<red_string("Failed to load file "+filename)<<std::endl;
        abort();
    }
    
    int t = doc.Type();
    if (t!=TiXmlNode::TINYXML_DOCUMENT)
    {
        std::cout<<red_string("Expected a document at the beginning of the file "+filename+", found something else.")<<std::endl;
        abort();
    }
    
    std::cout<<"    - CONFIG: " + cyan_string(config_file)<<std::endl;

    config = YAML::LoadFile(config_file);
    
    if(config.IsNull())
    {
	std::cout<<red_string("ERROR: cannot load configuration file")<<std::endl; 
	abort();
    }

    std::string urdf_filename, srdf_filename, joint_map_config;

    if(const char* robotology_root = std::getenv("ROBOTOLOGY_ROOT"))
    {
        if(!config["XBotInterface"].IsNull())
	{
	    if(config["XBotInterface"]["urdf_path"].IsNull())
	    {
		std::cout<<red_string("ERROR: urdf_path field is not in the configuration file")<<std::endl; 
		abort();
	    }
	    urdf_filename = std::string(robotology_root)+"/"+config["XBotInterface"]["urdf_path"].as<std::string>();
	    
	    if(config["XBotInterface"]["srdf_path"].IsNull())
	    {
		std::cout<<red_string("ERROR: srdf_path field is not in the configuration file")<<std::endl; 
		abort();
	    }
	    srdf_filename = std::string(robotology_root)+"/"+config["XBotInterface"]["srdf_path"].as<std::string>();
	    
	    if(config["XBotInterface"]["joint_map_path"].IsNull())
	    {
		std::cout<<red_string("ERROR: joint_map_path field is not in the configuration file")<<std::endl; 
		abort();
	    }
	    joint_map_config = std::string(robotology_root)+"/"+config["XBotInterface"]["joint_map_path"].as<std::string>();

	    std::cout<<"    - URDF: " + cyan_string(urdf_filename)<<std::endl;
	    std::cout<<"    - SRDF: " + cyan_string(srdf_filename)<<std::endl;
	    std::cout<<"    - JOINT_MAP_CONFIG: " + cyan_string(joint_map_config)<<std::endl;
	}
	else
	{
	    std::cout<<red_string("ERROR: XBotInterface field is not in the configuration file")<<std::endl; 
	    abort();
	}
    }
    else
    {
	std::cout<<red_string("ERROR: Robotology environment not sourced, can not retrieve files")<<std::endl; 
        abort();
    }

    if (!_XBotModel.init(urdf_filename,srdf_filename,joint_map_config))
    {
        std::cout<<red_string("ERROR: model initialization failed, please check the urdf_path and srdf_path in your YAML config file.")<<std::endl; 
        abort();
    }
    // generate the robot
    _XBotModel.generate_robot();

    _RobotInterface = XBot::RobotInterface::getRobot(config_file);

    for(auto chain_:_XBotModel.get_robot())
    {
	chains_q_sense[chain_.first];
	for(auto joint_:chain_.second)
	{
	    chains_q_sense.at(chain_.first)[_XBotModel.rid2Joint(joint_)]=0.0;
	}
    }

    // DEBUG 
    for(auto chain_:chains_q_sense)
    {
	std::cout<<" - "<<chain_.first<<std::endl;
	for(auto joint_:chain_.second)
	{
	    std::cout<<" -- "<<joint_.first<<" : "<<joint_.second<<std::endl;
	}
    }

    robot_widget.generateRobotWidgetFromModel(_XBotModel,_RobotInterface);

    tabs.addTab(&robot_widget,"Joints");
    
    std::cout<<std::endl<<" - Generating GUI..."<<std::endl;

    std::cout<<"    - Joints Control: " + cyan_string("ON")<<std::endl;
    
    #ifndef USING_ROS
    std::cout<<"    - PI:         " + purple_string("OFF")<<std::endl;
    #else

    pilot_interface.set_robot_name(_XBotModel.getName());

    std::cout<<"    - PI:         " + cyan_string("ON")<<std::endl;

    TiXmlElement* visualization=doc.FirstChildElement("visualization");
    if (visualization==NULL || visualization->Type()!=TiXmlNode::TINYXML_ELEMENT)
    {
        std::cout<<yellow_string("Could not find element visualization into file "+filename)<<std::endl;
    }
    else
    {
        std::cout<<"    - - visualization"<<std::endl;
	TiXmlElement* display = visualization->FirstChildElement("display");
	TiXmlElement* property;
	std::map<std::string,std::string> properties;	

	while(display)
	{
	    properties.clear();
	    std::string display_name = display->Attribute("name");
	    std::string display_type = display->Attribute("type");

	    std::cout<<"    - - | Display: "<<display_name<<" ( "<<display_type<<" )"<<std::endl;
	    
	    property = display->FirstChildElement("property");
	    
	    while(property)
	    {
		properties[property->Attribute("name")] = property->Attribute("value");

		std::cout<<"    - - | > "<<property->Attribute("name")<<" : "<<property->Attribute("value")<<std::endl;
		
		property = property->NextSiblingElement("property");
	    }

	    pilot_interface.add_display(display_name.c_str(),display_type.c_str(),properties);

	    display = display->NextSiblingElement("display");
	}

	TiXmlElement* frames = visualization->FirstChildElement("frames");
	TiXmlElement* frame = frames->FirstChildElement("frame");
	std::vector<std::string> frames_str;

	std::cout<<"    - - | frames"<<std::endl;

	while(frame)
	{
	    std::cout<<"    - - | > "<<frame->Attribute("name")<<std::endl;

	    frames_str.push_back(frame->Attribute("name"));

	    frame = frame->NextSiblingElement("frame");
	}

	pilot_interface.add_frames(frames_str);
    }

    TiXmlElement* modules=doc.FirstChildElement("modules");
    if (modules==NULL || modules->Type()!=TiXmlNode::TINYXML_ELEMENT)
    {
        std::cout<<yellow_string("Could not find element modules into file "+filename)<<std::endl;
    }
    else
    {
        std::cout<<"    - - modules"<<std::endl;
	TiXmlElement* module = modules->FirstChildElement("module");
	TiXmlElement* command;
	std::map<std::string,std::string> commands;

	while(module)
	{
	    commands.clear();
	    std::string module_name = module->Attribute("name");

	    std::cout<<"    - - | Module: "<<module_name<<std::endl;
	    
	    command = module->FirstChildElement("command");
	    
	    while(command)
	    {
		commands[command->Attribute("name")] = command->Attribute("type");

		std::cout<<"    - - | > "<<command->Attribute("name")<<" : "<<command->Attribute("type")<<std::endl;
		
		command = command->NextSiblingElement("command");
	    }

	    pilot_interface.add_module(module_name.c_str(),commands);

	    module = module->NextSiblingElement("module");
	}
    }

    TiXmlElement* interactive_markers=doc.FirstChildElement("interactive_markers");
    if (interactive_markers==NULL || interactive_markers->Type()!=TiXmlNode::TINYXML_ELEMENT)
    {
        std::cout<<yellow_string("Could not find element interactive_markers into file "+filename)<<std::endl;
    }
    else
    {
        std::cout<<"    - - interactive_markers"<<std::endl;
	TiXmlElement* marker = interactive_markers->FirstChildElement("marker");

	while(marker)
	{
	    std::string marker_name = marker->Attribute("name");

	    std::cout<<"    - - | Marker: "<<marker_name<<std::endl;

	    pilot_interface.add_interactive_marker(marker_name,object_count++);

	    marker = marker->NextSiblingElement("marker");
	}
    }
    
    tabs.addTab(&pilot_interface,"PI");
    
    #endif
    
    main_layout.addWidget(&tabs);

    connect(&sense_timer, SIGNAL(timeout()), this, SLOT(sense()));
    sense_timer.start(10);

    setLayout(&main_layout);
}

void XBot::GUI::sense()
{
    _RobotInterface->sense();

    for(auto& chain_ : chains_q_sense)
    {
        
	_RobotInterface->chain(chain_.first).getJointPosition(chain_.second);
    }

    robot_widget.setChainsJoints(chains_q_sense);
}

std::string XBot::GUI::getRobot()
{
    return _XBotModel.getName();
}

XBot::GUI::~GUI()
{

}