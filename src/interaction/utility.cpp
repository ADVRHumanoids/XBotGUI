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

#include "XBotGUI/interaction/utility.h"

XBot::widgets::utility::utility(std::string name_): name(name_)
{
    if(name=="trajectory_utils")
    {
	wid = new trajectory_utils_widget();
    }
    if(name=="manipulation_map")
    {
	wid = new manipulation_map_utils_widget();
    }

    main_layout.addWidget(wid);

    setLayout(&main_layout);
}

std::vector< display_property > XBot::widgets::utility::displays()
{
    if(name=="trajectory_utils")
	return ((trajectory_utils_widget*)wid)->displays;
    if(name=="manipulation_map")
	return ((trajectory_utils_widget*)wid)->displays;
    else
        return std::vector< display_property >();
}

XBot::widgets::utility::~utility()
{
    delete wid;
}