#include "XBotGUI/main_interface.h"
#include <iostream>

XBotGUI::main_interface::main_interface(): QWidget()
{
    test_button.setText("Test");
    main_layout.addWidget(&test_button);
    setLayout(&main_layout);
    
    connect(&test_button,SIGNAL(clicked(bool)),this,SLOT(test_slot()));
}

void XBotGUI::main_interface::test_slot()
{
    std::cout<<"test"<<std::endl;
}

XBotGUI::main_interface::~main_interface()
{

}