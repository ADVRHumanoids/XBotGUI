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
    std::string filename = "/external/XBotGUI/configs/xbotgui.xml";
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

	if(!config["XBotRTPlugins"].IsNull())
	{
	    if(!config["XBotRTPlugins"]["plugins"].IsNull())
	    {
	        std::vector<std::string> plugins = config["XBotRTPlugins"]["plugins"].as<std::vector<std::string>>();
	        
		for(auto plugin_name:plugins)
		{
		    plugin_names.push_back(plugin_name);
		}
	    }

	    if(plugin_names.size()==0) std::cout<<yellow_string("WARN: no XBotRTPlugins plugins found")<<std::endl; 
	}
	else
	{
	    std::cout<<yellow_string("WARN: XBotRTPlugins field is not in the configuration file")<<std::endl; 
	}
	
	if(!config["NRTPlugins"].IsNull())
        {
            if(!config["NRTPlugins"]["plugins"].IsNull())
            {
                std::vector<std::string> plugins = config["NRTPlugins"]["plugins"].as<std::vector<std::string>>();
                
                for(auto plugin_name:plugins)
                {
                    plugin_names.push_back(plugin_name);
                }
            }

            if(plugin_names.size()==0) std::cout<<yellow_string("WARN: no NRTPlugins plugins found")<<std::endl; 
        }
        else
        {
            std::cout<<yellow_string("WARN: NRTPlugins field is not in the configuration file")<<std::endl; 
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
    
    // get the first position read
    _RobotInterface->sense();
    
    // initialize second order filter
    Eigen::VectorXd current_q;
    _RobotInterface->getMotorPosition(current_q);
    _q_ref_filtered.reset(current_q);

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

    std::cout<<"    - "<<cyan_string("Active Plugins")<<":"<<std::endl;

    for(auto plugin_name:plugin_names)
    {
	std::cout<<"    - - "<<plugin_name<<std::endl;
    }
    
    std::cout<<"    - "<<cyan_string("Joints Control")<<": "<< green_string("ON")<<std::endl;
    
    #ifndef USING_ROS
    std::cout<<"    - "<<cyan_string("PI            ")<<":"  + red_string("OFF")<<std::endl;
    #else

    name_to_types["visualization_msgs::Marker::ARROW"] = visualization_msgs::Marker::ARROW;
    name_to_types["visualization_msgs::Marker::CUBE"] = visualization_msgs::Marker::CUBE;
    name_to_types["visualization_msgs::Marker::SPHERE"] = visualization_msgs::Marker::SPHERE;
    name_to_types["visualization_msgs::Marker::CYLINDER"] = visualization_msgs::Marker::CYLINDER;
    name_to_types["visualization_msgs::Marker::LINE_STRIP"] = visualization_msgs::Marker::LINE_STRIP;
    name_to_types["visualization_msgs::Marker::LINE_LIST"] = visualization_msgs::Marker::LINE_LIST;
    name_to_types["visualization_msgs::Marker::CUBE_LIST"] = visualization_msgs::Marker::CUBE_LIST;
    name_to_types["visualization_msgs::Marker::SPHERE_LIST"] = visualization_msgs::Marker::SPHERE_LIST;
    name_to_types["visualization_msgs::Marker::POINTS"] = visualization_msgs::Marker::POINTS;
    name_to_types["visualization_msgs::Marker::TEXT_VIEW_FACING"] = visualization_msgs::Marker::TEXT_VIEW_FACING;
    name_to_types["visualization_msgs::Marker::MESH_RESOURCE"] = visualization_msgs::Marker::MESH_RESOURCE;
    name_to_types["visualization_msgs::Marker::TRIANGLE_LIST"] = visualization_msgs::Marker::TRIANGLE_LIST;

    pilot_interface.set_robot_name(_XBotModel.getName());

    std::cout<<"    - "<<cyan_string("PI            ")<<":"  + green_string("ON")<<std::endl;

    TiXmlElement* visualization=doc.FirstChildElement("visualization");
    if (visualization==NULL || visualization->Type()!=TiXmlNode::TINYXML_ELEMENT)
    {
        std::cout<<yellow_string("Could not find element visualization into file "+filename)<<std::endl;
    }
    else
    {
        std::cout<<"    - - "<<purple_string("visualization")<<std::endl;
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
        std::cout<<"    - - "<<purple_string("modules additional commands")<<std::endl;
	TiXmlElement* module = modules->FirstChildElement("module");
	std::vector<std::vector<std::map<std::string,std::string>>> command_blocks;
	std::map<std::string,std::vector<std::string>> module_dependencies;

	while(module)
	{
	    command_blocks.clear();

	    std::string module_name = module->Attribute("name");

	    std::cout<<"    - - | Module: "<<module_name;

	    if(std::find(plugin_names.begin(),plugin_names.end(),module_name)==plugin_names.end())
	    {
		std::cout<<" ("<<yellow_string("not in Active Plugins, ignoring")<<")"<<std::endl;
		module = module->NextSiblingElement("module");
		continue;
	    }

	    std::cout<<std::endl;
	    
	    TiXmlElement* dependencies = module->FirstChildElement("dependencies");

	    if(dependencies)
	    {
		std::vector<std::map<std::string,std::string>> commands;
		TiXmlElement* dependency;

		dependency = dependencies->FirstChildElement("plugin");

		std::cout<<"    - - | > "<<"dependencies:"<<std::endl;

		while(dependency)
		{
		    std::string plugin_name = dependency->Attribute("name");
		    
		    if(std::find(plugin_names.begin(),plugin_names.end(),plugin_name)==plugin_names.end())
		    {
			std::cout<<" ("<<yellow_string("not in Active Plugins, ignoring")<<")"<<std::endl;
			dependency = dependency->NextSiblingElement("plugin");
			continue;
		    }

		    std::cout<<"    - - - | > "<<plugin_name<<std::endl;

		    module_dependencies[module_name].push_back(plugin_name);

		    dependency = dependency->NextSiblingElement("plugin");
		}
	    }

	    TiXmlElement* command_block = module->FirstChildElement("command_block");
	    
	    while(command_block)
	    {

	        std::vector<std::map<std::string,std::string>> commands;
		TiXmlElement* command;

		command = command_block->FirstChildElement("command");

		std::cout<<"    - - | > "<<"command group:"<<std::endl;

		while(command)
		{
		    std::map<std::string,std::string> command_attributes;
		    
		    command_attributes["type"] = std::string(command->Attribute("type"));
		    
		    std::cout<<"    - - - | > "<<command_attributes.at("type");

		    if(command_attributes.at("type")=="object_pose")
		    {
			command_attributes["topic"] = std::string(command->Attribute("topic"));
			command_attributes["interactive_marker"] = std::string(command->Attribute("interactive_marker"));

			std::cout<<std::endl<<"    - - - - | > topic: "<<command_attributes.at("topic");
			std::cout<<std::endl<<"    - - - - | > interactive_marker: "<<command_attributes.at("interactive_marker");
		    }
		    else if(command_attributes.at("type")=="goal")
		    {
			command_attributes["topic"] = std::string(command->Attribute("topic"));
			std::cout<<std::endl<<"    - - - - | > topic: "<<command_attributes.at("topic");

			std::map<std::string,std::string> properties;
			properties["Marker Topic"] = command_attributes.at("topic")+"_goal_marker";
			pilot_interface.add_display(module_name,"rviz/Marker",properties);
		    }
		    else if(command_attributes.at("type")=="object_sequence")
		    {
			command_attributes["topic"] = std::string(command->Attribute("topic"));
			command_attributes["interactive_markers_sequence"] = std::string(command->Attribute("interactive_markers_sequence"));
			std::cout<<std::endl<<"    - - - - | > topic: "<<command_attributes.at("topic");
			std::cout<<std::endl<<"    - - - - | > interactive_markers_sequence: "<<command_attributes.at("interactive_markers_sequence");
		    }
		    else if(command_attributes.at("type")=="cmd_service")
		    {
			command_attributes["name"] = std::string(command->Attribute("name"));
			std::cout<<std::endl<<"    - - - - | > name: "<<command_attributes.at("name");
		    }
		    else if(command_attributes.at("type")=="string")
		    {
			command_attributes["name"] = std::string(command->Attribute("name"));
			command_attributes["topic"] = std::string(command->Attribute("topic"));
			std::cout<<std::endl<<"    - - - - | > name: "<<command_attributes.at("name");
			std::cout<<std::endl<<"    - - - - | > topic: "<<command_attributes.at("topic");
		    }
		    else
		    {
			std::cout<<" ( "<<yellow_string("Undefined command type")<<" ) "<<std::endl;
			command = command->NextSiblingElement("command");
			continue;
		    }
		    
		    std::cout<<std::endl;
		    
		    commands.push_back(command_attributes);
		    
		    command = command->NextSiblingElement("command");
		}

		command_blocks.push_back(commands);

		command_block = command_block->NextSiblingElement("command_block");
	    }

	    if(module_dependencies.count(module_name))
		pilot_interface.add_module(module_name.c_str(),command_blocks,module_dependencies.at(module_name));
	    else
		pilot_interface.add_module(module_name.c_str(),command_blocks,std::vector<std::string>());

	    module = module->NextSiblingElement("module");
	}

	for(auto plugin:plugin_names)
	{
	    bool to_add=true;

	    for(auto dep:module_dependencies)
	    {
		if((std::find(dep.second.begin(),dep.second.end(),plugin)!=dep.second.end()))
		{
		    to_add=false;
		}
	    }

	    if(to_add) pilot_interface.add_module(plugin,std::vector<std::vector<std::map<std::string,std::string>>>(), std::vector<std::string>());
	}
    }

    TiXmlElement* utils=doc.FirstChildElement("utils");
    if (utils==NULL || utils->Type()!=TiXmlNode::TINYXML_ELEMENT)
    {
        std::cout<<yellow_string("Could not find element utils into file "+filename)<<std::endl;
    }
    else
    {
	std::cout<<"    - - "<<purple_string("utils")<<std::endl;
	TiXmlElement* util = utils->FirstChildElement("util");

	while(util)
	{
	    std::string util_name = util->Attribute("name");

	    std::cout<<"    - - | Util : "<<util_name;
	    
	    if(util_name!="trajectory_utils")
	    {
		std::cout<<" ( "<<yellow_string("Undefined util type")<<" ) "<<std::endl;
		util = util->NextSiblingElement("util");
		continue;
	    }

	    std::cout<<std::endl;

	    pilot_interface.add_utility(util_name);

	    util = util->NextSiblingElement("util");
	}
    }

    TiXmlElement* interactive_markers=doc.FirstChildElement("interactive_markers");
    if (interactive_markers==NULL || interactive_markers->Type()!=TiXmlNode::TINYXML_ELEMENT)
    {
        std::cout<<yellow_string("Could not find element interactive_markers into file "+filename)<<std::endl;
    }
    else
    {
        std::cout<<"    - - "<<purple_string("interactive_markers")<<std::endl;
	TiXmlElement* interactive_marker = interactive_markers->FirstChildElement("interactive_marker");

	while(interactive_marker)
	{
	    std::string interactive_marker_name = interactive_marker->Attribute("name");

	    std::cout<<"    - - | Interactive Marker: "<<interactive_marker_name<<std::endl;

	    std::map<std::string,object_properties> objects;

	    TiXmlElement* object = interactive_marker->FirstChildElement("object");
	    
	    while(object)
	    {
		std::string object_name = object->Attribute("name");

		TiXmlElement* property = object->FirstChildElement("property");

		objects[object_name];

		std::cout<<"    - - - | Object: "<<object_name<<std::endl;

		while(property)
		{
		    if(std::string(property->Attribute("name"))=="scale_x")
			objects.at(object_name).scale.x = std::atof(property->Attribute("value"));
		    if(std::string(property->Attribute("name"))=="scale_y")
			objects.at(object_name).scale.y = std::atof(property->Attribute("value"));
		    if(std::string(property->Attribute("name"))=="scale_z")
			objects.at(object_name).scale.z = std::atof(property->Attribute("value"));

		    if(std::string(property->Attribute("name"))=="type")
			objects.at(object_name).type = name_to_types.at(std::string(property->Attribute("value")));

		    if(std::string(property->Attribute("name"))=="mesh")
			objects.at(object_name).mesh_name = std::string(property->Attribute("value"));

		    if(std::string(property->Attribute("name"))=="id")
			objects.at(object_name).id = std::atoi(property->Attribute("value"));

		    std::cout<<"    - - - | > "<<property->Attribute("name")<<" : "<<property->Attribute("value")<<std::endl;

		    property = property->NextSiblingElement("property");
		}

		object = object->NextSiblingElement("object");
	    }

	    pilot_interface.add_interactive_marker(interactive_marker_name,objects);

	    interactive_marker = interactive_marker->NextSiblingElement("interactive_marker");
	}
    }
    
    TiXmlElement* interactive_markers_sequence=doc.FirstChildElement("interactive_markers_sequence");
    if (interactive_markers_sequence==NULL || interactive_markers_sequence->Type()!=TiXmlNode::TINYXML_ELEMENT)
    {
        std::cout<<yellow_string("Could not find element interactive_markers_sequence into file "+filename)<<std::endl;
    }
    else
    {
        std::cout<<"    - - "<<purple_string("interactive_markers_sequence")<<std::endl;
	TiXmlElement* interactive_marker = interactive_markers_sequence->FirstChildElement("interactive_marker");

	while(interactive_marker)
	{
	    std::string interactive_marker_name = interactive_marker->Attribute("name");

	    std::cout<<"    - - | Interactive Marker: "<<interactive_marker_name<<std::endl;

	    std::map<int,object_properties> objects;

	    TiXmlElement* object = interactive_marker->FirstChildElement("object");
	    
	    while(object)
	    {
		std::string object_name = object->Attribute("name");
                int object_id = std::atoi(object->Attribute("id"));

		TiXmlElement* property = object->FirstChildElement("property");

		objects[object_id];

		std::cout<<"    - - - | Object: "<<object_name<<" ID: "<<object_id<<std::endl;

		while(property)
		{
		    if(std::string(property->Attribute("name"))=="scale_x")
			objects.at(object_id).scale.x = std::atof(property->Attribute("value"));
		    if(std::string(property->Attribute("name"))=="scale_y")
			objects.at(object_id).scale.y = std::atof(property->Attribute("value"));
		    if(std::string(property->Attribute("name"))=="scale_z")
			objects.at(object_id).scale.z = std::atof(property->Attribute("value"));

		    if(std::string(property->Attribute("name"))=="type")
			objects.at(object_id).type = name_to_types.at(std::string(property->Attribute("value")));

		    if(std::string(property->Attribute("name"))=="mesh")
			objects.at(object_id).mesh_name = std::string(property->Attribute("value"));

		    std::cout<<"    - - - | > "<<property->Attribute("name")<<" : "<<property->Attribute("value")<<std::endl;

		    property = property->NextSiblingElement("property");
		}

		object = object->NextSiblingElement("object");
	    }

	    pilot_interface.add_interactive_marker_sequence(interactive_marker_name,objects);

	    interactive_marker = interactive_marker->NextSiblingElement("interactive_marker");
	}
    }

    tabs.addTab(&pilot_interface,"PI");
    
    #endif

    main_layout.addWidget(&tabs);

    // first sense and move to activate and let the robot where it is
    sense();
    usleep(10000);
    move();

    connect(&sense_timer, SIGNAL(timeout()), this, SLOT(sense()));
    sense_timer.start(20);
    connect(&move_timer, SIGNAL(timeout()), this, SLOT(move()));
    move_timer.start(5);
    
    // initialize second order filter TBD take it from config file
    _q_ref_filtered.setDamping(1.0);
    _q_ref_filtered.setOmega(10);    // TBD to tune
    _q_ref_filtered.setTimeStep(5);  // NOTE equal to the move calling frequency   

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

void XBot::GUI::move()
{
    robot_widget.getChainsJoints(chains_q_move);
    
    for(auto& chain_ : chains_q_move)
    {
	_RobotInterface->chain(chain_.first).setPositionReference(chain_.second);
    }

    // NOTE one filter for the whole robot
    Eigen::VectorXd current_q_ref;
    _RobotInterface->getPositionReference(current_q_ref);
    _RobotInterface->setPositionReference(_q_ref_filtered.process(current_q_ref));
    
    // actual move
    _RobotInterface->move();
}

std::string XBot::GUI::getRobot()
{
    return _XBotModel.getName();
}

XBot::GUI::~GUI()
{

}
