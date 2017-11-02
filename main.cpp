#include "robotact.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    RobotAct w;
    w.show();

    return a.exec();
}
