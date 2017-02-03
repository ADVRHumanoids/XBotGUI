#ifndef XBOTGUI_MAIN
#define XBOTGUI_MAIN

#include <QWidget>
#include <QPushButton>
#include <QBoxLayout>

namespace XBotGUI
{
class main_interface: public QWidget
{
Q_OBJECT
public:
    main_interface();
    ~main_interface();

private Q_SLOTS:
    void test_slot();
    
private:
    QPushButton test_button;

    QHBoxLayout main_layout;
};
}

#endif