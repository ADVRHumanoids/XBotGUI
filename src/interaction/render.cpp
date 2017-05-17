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
#include "XBotGUI/interaction/render.h"

XBot::widgets::render::render(): QWidget()
{
    render_panel_ = new rviz::RenderPanel();
    render_panel_->setMinimumHeight(30);
    render_panel_->setMinimumWidth(30);
    
    visualization_manager_ = new rviz::VisualizationManager( render_panel_ );
    render_panel_->initialize( visualization_manager_->getSceneManager(), visualization_manager_ );
    visualization_manager_->initialize();
    visualization_manager_->startUpdate();
    visualization_manager_->setFixedFrame("/base_link");
    
    main_layout.addWidget(render_panel_);

    setLayout(&main_layout);
}

void XBot::widgets::render::add_display(std::string name, std::string type, std::map< std::string, std::string > properties)
{
    displays[name] =  visualization_manager_->createDisplay( type.c_str(), name.c_str(), true );

    for(auto property:properties)
    {
	displays.at(name)->subProp(property.first.c_str())->setValue(property.second.c_str());
    }
}

XBot::widgets::render::~render()
{
    delete visualization_manager_;
    delete render_panel_;
}