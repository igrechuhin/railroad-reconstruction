#include <QtGui/QGuiApplication>
#include "qtquick2applicationviewer.h"

//#include "testwriter.h"
#include "point_cloud_processor.h"

#include <QtQml>

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);
//    qmlRegisterType<experiment::TestWriter>("ru.railroad.reconstruction.experiment", 1, 0, "TestWriter");
    qmlRegisterType<railroad::PointCloudProcessor>("ru.railroad.reconstruction.processor", 1, 1, "PCProcessor");


    QtQuick2ApplicationViewer viewer;
    viewer.setMainQmlFile(QStringLiteral("qml/qt-pcl/main.qml"));
    viewer.showExpanded();

    return app.exec();
}
