#include <QtGui/QGuiApplication>
#include "qtquick2applicationviewer.h"

#include "testwriter.h"

#include <QtQml>

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);
    qmlRegisterType<experiment::TestWriter>("ru.railroad.reconstruction.experiment", 1, 0, "TestWriter");


    QtQuick2ApplicationViewer viewer;
    viewer.setMainQmlFile(QStringLiteral("qml/qt-pcl/main.qml"));
    viewer.showExpanded();

    return app.exec();
}
