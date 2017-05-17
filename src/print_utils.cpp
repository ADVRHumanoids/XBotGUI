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

#include <string>
#include "XBotGUI/print_utils.h"

std::string blue_string(std::string s)
{
    std::string b="\033[0;94m";
    std::string n="\033[0m";
    return b+s+n;
}

std::string red_string(std::string s)
{
    std::string b="\033[0;91m";
    std::string n="\033[0m";
    return b+s+n;
}

std::string green_string(std::string s)
{
    std::string b="\033[0;92m";
    std::string n="\033[0m";
    return b+s+n;
}

std::string cyan_string(std::string s)
{
    std::string b="\033[0;96m";
    std::string n="\033[0m";
    return b+s+n;
}

std::string purple_string(std::string s)
{
    std::string b="\033[0;95m";
    std::string n="\033[0m";
    return b+s+n;
}

std::string yellow_string(std::string s)
{
    std::string b="\033[0;93m";
    std::string n="\033[0m";
    return b+s+n;
}