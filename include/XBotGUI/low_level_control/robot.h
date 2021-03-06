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

#ifndef XBOTGUI_ROBOT_H
#define XBOTGUI_ROBOT_H

#include <QWidget>
#include <QLabel>
#include <QBoxLayout>
#include <QTabWidget>
#include <map>
#include <unordered_map>

#include <XBotCoreModel.h>
#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/TypedefAndEnums.h>
#include <XBotGUI/interaction/module.h>

#include "chain.h"

namespace XBot
{
namespace widgets
{
class robot: public QWidget
{
Q_OBJECT
public:
    robot();
    ~robot();

    void generateRobotWidgetFromModel(XBotCoreModel& model, RobotInterface::Ptr robot_interface);
    void setChainsJoints(std::map<std::string,XBot::JointNameMap> chains_q_sense);
    void getChainsJoints(std::map<std::string,XBot::JointNameMap>& chains_q_move);

    void enableChainsJoints(bool enable_);

private Q_SLOTS:
    void on_xbot_communication_plugin_started_toggled();

private:
    QTabWidget tabs;
    QVBoxLayout main_layout;

    std::map<std::string,chain*> chains;

    module xbot_communication_plugin;
};
};
};

#endif