import QtQuick 2.2
import QtQuick.Controls 1.1
import QtQuick.Layouts 1.1
import QtQuick.Dialogs 1.1

import ru.railroad.reconstruction.processor 1.1

Rectangle {
    width: 300
    height: 400

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
            x: 0
            y: 0
            spacing: 0
            width: parent.width
            ToolButton { action: openAction }
        }
    }

    Component {
        id: highlight
        Rectangle {
            width: listView.width
            height: 30
            color: "lightsteelblue"
            y: listView.currentItem.y
            Behavior on y {
                SpringAnimation {
                    spring: 3
                    damping: 0.2
                }
            }
        }
    }

    Component {
       id: contactDelegate

       Item {
           id: listItem
           width: listView.width
           height: 30

           Text {
               verticalAlignment: Text.AlignBottom
               text: '<b>'+index+'</b>: ' + modelData
               wrapMode: Text.WrapAnywhere
           }

           MouseArea {
               anchors.fill: parent
               onClicked: {
                   listView.currentIndex = index
                   parent.forceActiveFocus()
               }
           }
       }
    }

    ListView {
        id: listView
        x: 0
        y: 50
        width: parent.width
        height: parent.height - 50
        highlight: highlight
        focus: true
        model: pcProcessor.status
        delegate: contactDelegate
    }
}
