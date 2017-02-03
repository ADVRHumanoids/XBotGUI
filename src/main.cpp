#include "XBotGUI/main_interface.h"
#include <QApplication>
#include <qicon.h>

int main(int argc, char** argv)
{
    QApplication app(argc,argv);

    XBotGUI::main_interface gui;
  
    gui.show();
    gui.setWindowTitle("XBotGUI");
    gui.setWindowIcon(QIcon("resources/logo.png"));
  
    return app.exec();
}