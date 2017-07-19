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

#include <XBotGUI/utils/object_properties.h>

XBot::object_properties::object_properties()
{
    scale.x=1.0;
    scale.y=1.0;
    scale.z=1.0;
    pose.orientation.w=1;
}

void XBot::object_properties::print()
{
    std::cout<<"Name: "<<name<<std::endl;
    
    std::cout<<"ID: "<<id<<std::endl;
    
    std::cout<<"Scale: "<<scale.x<<" | "<<scale.y<<" | "<<scale.z<<std::endl;

    std::cout<<"Position: "<<pose.position.x<<" | "<<pose.position.y<<" | "<<pose.position.z<<std::endl;
    std::cout<<"Orientation: "<<pose.orientation.x<<" | "<<pose.orientation.y<<" | "<<pose.orientation.z<<" | "<<pose.orientation.w<<std::endl;

    std::cout<<"Marker Type: "<<type<<std::endl;
    std::cout<<"Mesh Name: "<<mesh_name<<std::endl;
}