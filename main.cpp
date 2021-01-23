#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    qRegisterMetaType< std::vector<cv::Point3f> >();

    MainWindow w;
    w.show();
    return a.exec();
}
