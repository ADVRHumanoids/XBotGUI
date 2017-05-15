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

#ifndef XBOTGUI_CHAIN_H
#define XBOTGUI_CHAIN_H

#include <QWidget>
#include <QLabel>
#include <QBoxLayout>
#include <QGridLayout>
#include <map>
#include <urdf_parser/urdf_parser.h>
#include <XBotInterface/RobotInterface.h>

#include "joint.h"

namespace XBot
{
namespace widgets
{
class chain: public QWidget
{
Q_OBJECT
public:
    chain(std::string name_,std::vector<std::string> joint_names, boost::shared_ptr<urdf::ModelInterface const> urdf, std::map<std::string,XBot::ControlMode> control_map);
    ~chain();
    void setJoints(XBot::JointNameMap joints_q_sense);

private:
    std::map<std::string,joint*> joints;
    QGridLayout main_layout;

    std::string name;
};
};
};

#endif