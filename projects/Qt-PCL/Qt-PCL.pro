# Add more folders to ship with the application, here
qmlFolder.source = qml/qt-pcl
qmlFolder.target = qml

imagesFolder.source = images

DEPLOYMENTFOLDERS = qmlFolder imagesFolder

# Additional import path used to resolve QML modules in Creator's code model
QML_IMPORT_PATH =

# The .cpp file which was generated for your project. Feel free to hack it.
SOURCES += main.cpp \
    testwriter.cpp \
    pointcloudprocessor.cpp

# Installation path
# target.path =

# Please do not modify the following two lines. Required for deployment.
include(qtquick2applicationviewer/qtquick2applicationviewer.pri)
qtcAddDeployment()

HEADERS += \
    testwriter.h \
    pointcloudprocessor.h

macx: INCLUDEPATH += \
    /usr/local/include/pcl-1.6 \
    /opt/local/include/eigen3 \
    /opt/local/include

macx: LIBS += \
    -L/opt/local/lib \
    -L/usr/local/lib \

LIBS += \
    -lpcl_io_ply \
    -lpcl_io \
    -lpcl_common \
    -lusb-1.0.0

RESOURCES += \
    resources.qrc
