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

#include "XBotGUI/low_level_control/chain.h"

XBot::widgets::chain::chain(std::string name_, std::vector< std::string > joint_names, boost::shared_ptr<urdf::ModelInterface const> urdf, std::map<std::string,XBot::ControlMode> control_map): QWidget()
{
    name = name_;

    int r = 0;
    int c = 0;
    for(auto joint_:joint_names)
    {
        joint* j = new joint(joint_,urdf->getJoint(joint_),control_map.at(joint_));
	joints[joint_] = j;
	main_layout.addWidget(j,r,c);
	c++;
	if(c==4)
	{
	    r++;
	    c=0;
	}
    }
    
    setLayout(&main_layout);
}

void XBot::widgets::chain::enableJoints(bool enable_)
{
    for(auto joint_:joints)
    {
	joint_.second->enable(enable_);
    }
}

void XBot::widgets::chain::setJoints(XBot::JointNameMap joints_q_sense)
{
    for(auto joint_:joints_q_sense)
    {
	joints.at(joint_.first)->set(joint_.second);
    }
}

void XBot::widgets::chain::getJoints(XBot::JointNameMap& joints_q_move)
{
    for(auto& joint_:joints)
    {
        joints.at(joint_.first)->get(joints_q_move[joint_.first]);
    }
}

XBot::widgets::chain::~chain()
{
    for(auto j:joints)
    {
	delete j.second;
    }
}