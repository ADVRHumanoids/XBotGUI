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

#ifndef XBOTGUI_MAIN_H
#define XBOTGUI_MAIN_H

#include <string>

#include <QWidget>
#include <QPushButton>
#include <QBoxLayout>
#include <QTimer>
#include <QSocketNotifier>

#include <yaml-cpp/yaml.h>
#include <XBotCoreModel.h>
#include <XBotInterface/Utils.h>
#include <XBotInterface/RobotInterface.h>

#include "print_utils.h"
#include "low_level_control/robot.h"
#include "cmake_options.h"

#ifndef USING_ROS
#else
#include "interaction/pi.h"
#include "interaction/status_manager.h"
#endif

namespace XBot
{
class GUI: public QWidget
{
Q_OBJECT
public:
    GUI(std::string config_file);
    ~GUI();

    std::string getRobot();

    //Unix signal handler
    static void intSignalHandler(int);

private:
    widgets::robot robot_widget;
    QHBoxLayout main_layout;
    QTabWidget tabs;
    
    YAML::Node config;
    XBotCoreModel _XBotModel;
    RobotInterface::Ptr _RobotInterface;

    QTimer sense_timer;
    std::map<std::string,XBot::JointNameMap> chains_q_sense;
    QTimer move_timer;
    std::map<std::string,XBot::JointNameMap> chains_q_move;

    #ifndef USING_ROS
    #else
    std::map<std::string, visualization_msgs::Marker::Type::_type_type> name_to_types;
    widgets::pi pilot_interface;
    status_manager status_manager_;
    #endif
    
    XBot::Utils::SecondOrderFilter<Eigen::VectorXd> _q_ref_filtered;

    std::vector<std::string> plugin_names;

    char separator_symbol;
    std::string fix_double_separator(std::string str);

    static int sigintFd[2];
    QSocketNotifier* snInt;

public Q_SLOTS:
    void handleSigInt();
    
private Q_SLOTS:
    void sense();
    void move();

};
}

#endif