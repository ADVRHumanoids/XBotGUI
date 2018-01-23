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

#include <QApplication>
#include <qicon.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>

#include <XBotInterface/Logger.hpp>
#include <XBotInterface/RtLog.hpp>

#include <XBotCore-interfaces/XBotOptions.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

bool getDefaultConfig(std::string& config) 
{
    config = XBot::Utils::getXBotConfig();
    if(config == "") {
        return false;
    }
    return true;
}

int main(int argc, char** argv)
{
    std::cout<<std::endl<<" - Starting XBotGUI..."<<std::endl;

    #ifndef USING_ROS
    #else
    if(!ros::isInitialized())
    {
	ros::init(argc,argv,"XBotGUI");
    }
    #endif
    
    std::string config_file = "";
    
    QApplication app(argc,argv);

    XBot::Options options;
    
    {
        po::options_description desc("XBotGUI. Available options");
        desc.add_options()
            ("config,C", po::value<std::string>(),"Path to custom config file. If not set, a default config file must be configured via set_xbot_config.")
            ("verbose,V", "Verbose mode.")
            ("help", "Shows this help message.")
            
        ;

        
        po::positional_options_description p;
        p.add("config", -1);

        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).
                options(desc).positional(p).run(), vm);
        po::notify(vm);
        
        if(vm.count("help")){
            std::cout << desc << std::endl;
            return 0;
        }
        
        if(vm.count("verbose")){
            XBot::Logger::SetVerbosityLevel(XBot::Logger::Severity::LOW);
            XBot::Logger::info("Verbose mode ON \n");
        }
        else{
            XBot::Logger::SetVerbosityLevel(XBot::Logger::Severity::MID);
        }
        
        if(vm.count("config")) {
            config_file = fs::absolute(vm["config"].as<std::string>()).string();
        }
        else{
            if(!getDefaultConfig(config_file)){
                std::cout << desc << std::endl;
                return -1;
            }
        }
        
    }


    XBot::Logger::info(XBot::Logger::Severity::HIGH) << XBot::bold_on << "XBotCore using config file " << config_file << XBot::Logger::endl();


    XBot::GUI gui(config_file);
  
    gui.show();
    gui.setWindowTitle(QString::fromStdString("XBotGUI : " + gui.getRobot()));
    gui.setWindowIcon(QIcon("resources/logo.png"));

    std::cout<<std::endl;
    std::cout<<green_string(" >> Started XBotGUI for robot ")<<cyan_string(gui.getRobot())<<green_string(" <<")<<std::endl<<std::endl;

    #ifndef USING_ROS
    #else
    ros::Time start = ros::Time::now();

    tf::TransformBroadcaster dummy_broadcaster;
    tf::Transform dummy_transform;
    tf::transformKDLToTF(KDL::Frame::Identity(),dummy_transform);
    
    while(ros::Time::now()-start<ros::Duration(1.0))
    {
	dummy_broadcaster.sendTransform(tf::StampedTransform(dummy_transform, ros::Time::now(), "/XBotGUI_root", "/base_link"));
    }
    #endif

    // handling unix signals
    struct sigaction sigint;
    sigint.sa_handler = XBot::GUI::intSignalHandler;
    sigemptyset(&sigint.sa_mask);
    sigint.sa_flags |= SA_RESTART;
    if (sigaction(SIGINT, &sigint, 0) > 0) std::cout<<"ERROR: Something wrong with unix interrupt signal handling"<<std::endl;
    
    return app.exec();
}