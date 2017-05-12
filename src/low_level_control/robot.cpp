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

#include "XBotGUI/low_level_control/robot.h"

XBot::widgets::robot::robot(): QWidget()
{

}

void XBot::widgets::robot::generateRobotWidgetFromModel(XBotCoreModel& model, RobotInterface::Ptr robot_interface)
{
    std::map<std::string,XBot::ControlMode> control_map ;
    robot_interface->getControlMode(control_map);
   
    for(auto chain_:model.get_robot())
    {
        std::vector<std::string> joint_names;
	for(auto j:chain_.second) joint_names.push_back(model.rid2Joint(j));
        chain* c = new chain(chain_.first,joint_names,model.get_urdf_model(), control_map);
	chains[chain_.first] = c;
	tabs.addTab(c,QString::fromStdString(chain_.first));
    }
    
    main_layout.addWidget(&tabs);
    setLayout(&main_layout);
}

XBot::widgets::robot::~robot()
{
    for(auto c:chains)
    {
	delete c.second;
    }
}