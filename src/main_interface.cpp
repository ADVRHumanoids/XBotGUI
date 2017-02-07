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
	std::cout<<red_string("ERROR: Robotology environment not sourced, can not retrive files")<<std::endl; 
        abort();
    }

    if (!_XBotModel.init(urdf_filename,srdf_filename,joint_map_config))
    {
        std::cout<<red_string("ERROR: model initialization failed, please check the urdf_path and srdf_path in your YAML config file.")<<std::endl; 
        abort();
    }
    // generate the robot
    _XBotModel.generate_robot();

    robot_widget.generateRobotWidgetFromModel(_XBotModel);

    main_layout.addWidget(&robot_widget);
    setLayout(&main_layout);    
}

std::string XBot::GUI::getRobot()
{
    return _XBotModel.getName();
}

XBot::GUI::~GUI()
{

}