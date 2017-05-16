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

    tabs.addTab(&robot_widget,"Robot");
    
    std::cout<<std::endl<<" - Generating GUI..."<<std::endl;

    std::cout<<"    - Joints Control: " + cyan_string("ON")<<std::endl;
    
    #ifndef BUILD_ROBOT_RENDER
    std::cout<<"    - Render:         " + purple_string("OFF")<<std::endl;
    #else

    std::cout<<"    - Render:         " + cyan_string("ON")<<std::endl;

    TiXmlElement* displays=doc.FirstChildElement("displays");
    if (displays==NULL || displays->Type()!=TiXmlNode::TINYXML_ELEMENT)
    {
        std::cout<<yellow_string("Could not find element displays into file "+filename)<<std::endl;
    }
    else
    {
        std::cout<<"    - - displays"<<std::endl;
	TiXmlElement* display = displays->FirstChildElement("display");
	TiXmlElement* property;
	std::map<std::string,std::string> properties;	

	while(display)
	{
	    properties.clear();
	    std::string display_name = display->Attribute("name");
	    std::string display_type = display->Attribute("type");

	    std::cout<<"    - - > "<<display_name<<" ( "<<display_type<<" )"<<std::endl;
	    
	    property = display->FirstChildElement("property");
	    
	    while(property)
	    {
		properties[property->Attribute("name")] = property->Attribute("value");

		std::cout<<"    - - |> "<<property->Attribute("name")<<" : "<<property->Attribute("value")<<std::endl;
		
		property = property->NextSiblingElement("property");
	    }

	    robot_render.add_display(display_name.c_str(),display_type.c_str(),properties);

	    display = display->NextSiblingElement("display");
	}
    }
    
    tabs.addTab(&robot_render,"Render");
    
    #endif
    
    main_layout.addWidget(&tabs);

    connect(&sense_timer, SIGNAL(timeout()), this, SLOT(sense()));
    sense_timer.start(10);

    setLayout(&main_layout);
}

void XBot::GUI::sense()
{
    _RobotInterface->sense();

    for(auto chain_:chains_q_sense)
    {
	_RobotInterface->getJointPosition(chain_.second);
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