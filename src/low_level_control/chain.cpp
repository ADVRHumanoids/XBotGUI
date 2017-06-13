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

XBot::widgets::chain::chain(std::string name_, std::vector< std::string > joint_names, std::map<std::string,int> joint_names_id, boost::shared_ptr<urdf::ModelInterface const> urdf, std::map<std::string,XBot::ControlMode> control_map): QWidget()
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

    plot_button.setText("Plot");
    
    main_layout.addWidget(&plot_button,r+1,0);

    connect(&plot_button,SIGNAL(clicked()),this,SLOT(on_plot_button_clicked()));

    plot_process_string = "rosrun rqt_plot rqt_plot ";

    for(auto name:joint_names)
    {
	plot_process_string = plot_process_string + "/xbotcore/centauro/joint_states/link_position[" + QString::number(joint_names_id.at(name)) + "] ";
    }

    setLayout(&main_layout);
}

void XBot::widgets::chain::on_plot_button_clicked()
{	
    if(plot_process.state()==QProcess::Running || plot_process.state()==QProcess::Starting)
    {
	plot_process.terminate();
	plot_process.waitForFinished();
    }

    plot_process.start(plot_process_string);
    plot_process.waitForStarted(-1);
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