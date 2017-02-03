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

XBotGUI::main_interface::main_interface(std::string config_file): QWidget()
{
    config = YAML::LoadFile(config_file);
  
    test_button.setText("Test");
    main_layout.addWidget(&test_button);
    setLayout(&main_layout);
    
    connect(&test_button,SIGNAL(clicked(bool)),this,SLOT(test_slot()));
}

std::string XBotGUI::main_interface::getRobot()
{
    if(config["x_bot_interface"])
    {
	if(config["x_bot_interface"]["urdf_filename"])
	{
	    return "test"; //TODO parse urdf
	}
    }

    std::cout<<red_string("CAN NOT FIND ROBOT NAME")<<std::endl;
    return "unknown robot";
}

void XBotGUI::main_interface::test_slot()
{
    std::cout<<"test"<<std::endl;
}

XBotGUI::main_interface::~main_interface()
{

}