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
#ifndef XBOTGUI_COMMAND_WIDGET_H
#define XBOTGUI_COMMAND_WIDGET_H

#include <QWidget>
#include <QBoxLayout>
#include <string>

namespace XBot
{
namespace widgets
{
class command_widget: public QWidget
{
public:
	virtual void set_fixed_frame(std::string frame);

	virtual std::string get_type();
	
	virtual std::vector<std::string> get_enabling_status();

	virtual void set_enabling_status(std::string status);

	std::vector<std::string> enabling_status;
	
	QVBoxLayout main_layout;
};
}
}

#endif