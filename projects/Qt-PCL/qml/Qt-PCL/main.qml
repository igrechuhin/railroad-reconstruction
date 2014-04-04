import QtQuick 2.2
import QtQuick.Controls 1.1
import QtQuick.Layouts 1.1
import QtQuick.Dialogs 1.1

//import ru.railroad.reconstruction.experiment 1.0

import ru.railroad.reconstruction.processor 1.0

Rectangle {
    width: 640
    height: 400

    Text {
        text: qsTr("Hello World")
        anchors.centerIn: parent
    }

//        TestWriter {
//            id: testWriter
//        }

//        MouseArea {
//            anchors.fill: parent
//            onClicked: {
//                testWriter.writePCD();
//    //            Qt.quit();
//            }
//        }

    PCProcessor { id: pcProcessor }
    FileDialog {
        id: modelOpenDialog
        title: qsTr("Select dots model")
        selectExisting: true
        selectFolder: false
        selectMultiple: false
        nameFilters: [ "Dots model (*.csv)", "PCL model (*.pcd)" ]
        onAccepted: pcProcessor.readModel(fileUrl)
    }

    Action {
        id: openAction
        text: qsTr("&Open")
        shortcut: StandardKey.Open
        iconSource: "qrc:/images/document-open.png"
        onTriggered: modelOpenDialog.open()
        tooltip: qsTr("Open model")
    }

    ToolBar {
        id: toolbar
        x: 0
        y: 0
        width: parent.width
        height: 50
        RowLayout {
            id: toolbarLayout
            x: -6
            y: 3
            spacing: 0
            width: parent.width
            ToolButton { action: openAction }
        }
    }

    StatusBar {
        id: statusbar

        anchors {
            bottom: parent.bottom
            left: parent.left
            right: parent.right
        }
        RowLayout {
            Text {
                id: statusText
                text: pcProcessor.status
                anchors.centerIn: parent
            }
        }
    }
}
