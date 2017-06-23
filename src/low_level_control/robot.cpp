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

XBot::widgets::robot::robot(): QWidget(), xbot_communication_plugin("XBotCommunicationPlugin",std::vector<std::map<std::string,std::string>>())
{

}

void XBot::widgets::robot::enableChainsJoints(bool enable_)
{
    for(auto& chain_:chains)
    {
        chains.at(chain_.first)->enableJoints(enable_);
    }
}

void XBot::widgets::robot::on_xbot_communication_plugin_started_toggled()
{
    enableChainsJoints(xbot_communication_plugin.get_switch_button()->isChecked());
}

void XBot::widgets::robot::generateRobotWidgetFromModel(XBotCoreModel& model, RobotInterface::Ptr robot_interface)
{
    std::map<std::string,XBot::ControlMode> control_map ;
    robot_interface->getControlMode(control_map);
   
    std::vector<std::string> enabled_joint_names = robot_interface->getEnabledJointNames();
    std::map<std::string,int> joint_names_id;
    for(int i=0;i<enabled_joint_names.size();i++)
    {
	joint_names_id[enabled_joint_names.at(i)] = i;
    }

    for(auto chain_:model.get_robot())
    {
	std::vector<std::string> joint_names;
	for(auto j:chain_.second)
	{
	    joint_names.push_back(model.rid2Joint(j));
	}
        chain* c = new chain(chain_.first,joint_names,joint_names_id,model.get_urdf_model(), control_map);
	chains[chain_.first] = c;
	tabs.addTab(c,QString::fromStdString(chain_.first));
    }

    main_layout.addWidget(&xbot_communication_plugin);
    main_layout.addWidget(&tabs);
    setLayout(&main_layout);

    connect((xbot_communication_plugin.get_switch_button()),SIGNAL(clicked()),this,SLOT(on_xbot_communication_plugin_started_toggled()));

    enableChainsJoints(false);
}

void XBot::widgets::robot::setChainsJoints(std::map< std::string, XBot::JointNameMap > chains_q_sense)
{
    for(auto chain_:chains_q_sense)
    {
        chains.at(chain_.first)->setJoints(chain_.second);
    }
}

void XBot::widgets::robot::getChainsJoints(std::map< std::string, XBot::JointNameMap >& chains_q_move)
{
    for(auto& chain_:chains)
    {
        chains.at(chain_.first)->getJoints(chains_q_move[chain_.first]);
    }
}

XBot::widgets::robot::~robot()
{
    for(auto c:chains)
    {
	delete c.second;
    }
}