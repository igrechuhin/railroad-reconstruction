import QtQuick 2.2
import QtQuick.Controls 1.1
import QtQuick.Layouts 1.1
import QtQuick.Dialogs 1.1
import QtQuick.Controls.Styles 1.1

import ru.railroad.reconstruction.processor 1.1

Rectangle {
    width: 500
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

    FileDialog {
        id: modelSaveDialog
        title: qsTr("Select file to save model")
        selectExisting: false
        selectFolder: false
        selectMultiple: false
        nameFilters: [ "PCL model (*.pcd)" ]
        onAccepted: pcProcessor.writeModel(fileUrl, cloudSelection.currentText)
    }

    Action {
        id: openAction
        text: qsTr("Open model")
        shortcut: StandardKey.Open
        onTriggered: modelOpenDialog.open()
        tooltip: qsTr("Open model")
    }

    Action {
        id: saveAction
        text: qsTr("Save model")
        shortcut: StandardKey.Save
        onTriggered: modelSaveDialog.open()
        tooltip: qsTr("Save model")
    }

    Action {
        id: euclideanClusterExtractionAction
        text: qsTr("Euclidean Cluster Extraction")
        onTriggered: pcProcessor.euclideanClusterExtraction(cloudSelection.currentText)
        tooltip: qsTr("Euclidean Cluster Extraction")
    }

    Component {
        id: btnStyle
        ButtonStyle {
            background: Rectangle {
                border.width: 2
                border.color: "#888"
                radius: 4
                gradient: Gradient {
                    GradientStop { position: 0 ; color: control.pressed ? "#ccc" : "#eee" }
                    GradientStop { position: 1 ; color: control.pressed ? "#aaa" : "#ccc" }
                }
            }
        }
    }

    ToolBar {
        id: toolbar
        x: 0
        y: 0
        width: toolbarLayout.childrenRect.width + 12
        height: parent.height
        ColumnLayout {
            id: toolbarLayout
            x: 0
            y: 0
            spacing: 5
            ToolButton {
                action: openAction
                style: btnStyle
            }
            ComboBox {
                id: cloudSelection
                Layout.fillWidth: true
                model: pcProcessor.clouds
                onCurrentTextChanged: pcProcessor.draw(cloudSelection.currentText)
            }
            ToolButton {
                action: saveAction
                visible: cloudSelection.count != 0
                style: btnStyle
            }
            ToolButton {
                action: euclideanClusterExtractionAction
                visible: cloudSelection.count != 0
                style: btnStyle
            }
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
        x: toolbarLayout.childrenRect.width + 12
        y: 0
        width: parent.width - x
        height: parent.height
        highlight: highlight
        focus: true
        model: pcProcessor.status
        delegate: contactDelegate
    }
}
