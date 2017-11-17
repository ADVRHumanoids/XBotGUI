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

#include <XBotGUI/utils/command_widget.h>
#include <sstream>

void XBot::widgets::command_widget::set_fixed_frame(std::string frame)
{

}

std::string XBot::widgets::command_widget::get_type()
{
    return "command_widget";
}

void XBot::widgets::command_widget::set_enabling_status(std::string status)
{
    std::istringstream source(status);

    std::string state;

    while(std::getline(source,state,' '))
    {
	    enabling_status.push_back(state);
    }
}

std::vector<std::string> XBot::widgets::command_widget::get_enabling_status()
{
    return enabling_status;
}