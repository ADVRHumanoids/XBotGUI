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

#include "XBotGUI/utils/trajectory_utils_widget.h"
#include "XBotGUI/print_utils.h"
#include <iostream>
#include <cstdlib>

XBot::widgets::trajectory_utils_widget::trajectory_utils_widget()
{
    switch_button.setCheckable(true);
    switch_button.setText("Start Traj Utils");

    main_layout.addWidget(&switch_button);

    connect(&switch_button,SIGNAL(clicked()),this,SLOT(on_switch_button_clicked()));

    setLayout(&main_layout);

    std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");

    std::string path = robotology_root + "/external/trajectory_utils/launch/";
    QString command = QString::fromStdString("roslaunch " + path + "TrajXBotGui.launch");
    traj_utils_processes[command];
    
    path = robotology_root + "/external/ManipulationPlugin/python/";
    command = QString::fromStdString("python " + path + "cartesianTrjParser2.py /LSoftHand_segments /segment_control");
    traj_utils_processes[command];
    
    command = QString::fromStdString("python " + path + "cartesianTrjParser2.py /RSoftHand_segments /segment_control");
    traj_utils_processes[command];
}

void XBot::widgets::trajectory_utils_widget::on_switch_button_clicked()
{
    if(switch_button.isChecked())
    {
	switch_button.setText("Stop Traj Utils");

	for(auto& process:traj_utils_processes)
	{
	    if(process.second.state()==QProcess::Running || process.second.state()==QProcess::Starting)
	    {
		process.second.terminate();
		process.second.waitForFinished();
	    }

	    process.second.start(process.first);
	    process.second.waitForStarted(-1);
	}

	std::cout<<green_string("[ UTIL trajectory_utils ") + yellow_string("STARTED") + green_string(" ]")<<std::endl;
    }
    else
    {
	switch_button.setText("Start Traj Utils");

	for(auto& process:traj_utils_processes)
	{
	    if(process.second.state()!=QProcess::NotRunning)
	    {
		process.second.terminate();
		process.second.waitForFinished();
	    }
	}
	
	std::cout<<green_string("[ UTIL trajectory_utils ") + yellow_string("STOPPED") + green_string(" ]")<<std::endl;
    }
}

XBot::widgets::trajectory_utils_widget::~trajectory_utils_widget()
{

}