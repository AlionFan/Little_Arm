import QtQuick 2.15
import QtQuick3D
import QtQuick3D.Helpers

View3D {
    id: view3D
    implicitWidth: 640
    implicitHeight: 480
    renderMode: View3D.Offscreen

    environment: sceneEnvironment
    SceneEnvironment {
        id: sceneEnvironment
        antialiasingMode: SceneEnvironment.MSAA
        antialiasingQuality: SceneEnvironment.High
        backgroundMode: SceneEnvironment.Color
        clearColor: "white"
    }


    // grid
    AxisHelper {
        enableAxisLines: true
        enableXYGrid: true
        enableXZGrid: true
        enableYZGrid: true
        gridColor: "black"
        gridOpacity: 0.1
    }

    
    // Camera
    Node{
        id: cameraNode
        PerspectiveCamera{
            id: camera
            x: 0
            y: 0
            z: 1500
            eulerRotation.x: 0
            eulerRotation.y: 0
            eulerRotation.z: 0

            clipNear: 0
            clipFar: 3000
            fieldOfView: 60
            fieldOfViewOrientation: Camera.Horizontal
        }
    }

    // Mouse Ctrl
    MouseArea{
        anchors.fill: parent
        acceptedButtons: Qt.LeftButton | Qt.RightButton | Qt.MiddleButton
        hoverEnabled: true
        property int middleCx: 0
        property int middleCy: 0
        property int rightCx: 0
        property int rightCy: 0
        property bool middlePress: false
        property bool rightPress: false

        onEntered: {
            cursorShape = Qt.OpenHandCursor
        }
        onExited: {
            cursorShape = Qt.ArrowCursor
        }

        onPressed: {
            if (mouse.button === Qt.MiddleButton)
            {
                cursorShape = Qt.CrossCursor
                middleCx = mouse.x
                middleCy = mouse.y
                middlePress = true
            }
            else if (mouse.button === Qt.RightButton)
            {
                cursorShape = Qt.ClosedHandCursor
                rightCx = mouse.x
                rightCy = mouse.y
                rightPress = true
            }
            else if (mouse.button === Qt.LeftButton)
            {
                cursorShape = Qt.PointingHandCursor
            }
        }
        onPositionChanged: {
            if (middlePress) {
                let intercalX = mouse.x - middleCx
                let intercalY = mouse.y - middleCy
                cameraNode.eulerRotation.x = cameraNode.eulerRotation.x - intercalY
                cameraNode.eulerRotation.y = cameraNode.eulerRotation.y - intercalX
                middleCx = mouse.x
                middleCy = mouse.y
            }
            if (rightPress) {
                let intervolX = mouse.x - rightCx
                let intervolY = mouse.y - rightCy
                camera.x = camera.x - (0.000027 * intervolX * camera.z * camera.fieldOfView)
                camera.y = camera.y + (0.000027 * intervolY * camera.z * camera.fieldOfView)
                rightCx = mouse.x
                rightCy = mouse.y
            }
        }
        onReleased: {
            cursorShape = Qt.OpenHandCursor
            if (mouse.button === Qt.MiddleButton) {
                middlePress = false
            }else if (mouse.button === Qt.RightButton)
            {
                rightPress = false
            }
        }

        onWheel: {
            if (wheel.angleDelta.y > 0)
                camera.z = camera.z * 1.1
            else
                camera.z = camera.z * 0.9
        }
    }
}