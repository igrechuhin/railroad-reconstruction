import QtQuick 2.0
import ru.railroad.reconstruction.experiment 1.0

Rectangle {
    width: 360
    height: 360
    Text {
        text: qsTr("Hello World")
        anchors.centerIn: parent
    }

    TestWriter {
        id: testWriter
    }

    MouseArea {
        anchors.fill: parent
        onClicked: {
            testWriter.writePCD();
//            Qt.quit();
        }
    }
}
