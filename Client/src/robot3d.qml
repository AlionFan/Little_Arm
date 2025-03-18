import QtQuick 2.15
import QtQuick3D
import QtQuick3D.Helpers

View3D {
    // 添加属性以接收从C++传递的关节角度值
    property int joint1Angle: 0
    property int joint2Angle: 0
    property int joint3Angle: 0
    property int joint4Angle: 0
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

    // 添加根节点，设置Z轴朝上，X轴指向右侧，Y轴指向屏幕外的坐标系
    Node {
        id: rootNode
        // 旋转使Z轴朝上，X轴指向右侧，Y轴指向屏幕外（右手坐标系）
        eulerRotation.x: -90
        eulerRotation.y: 0
        eulerRotation.z: 0
        
        // grid
        AxisHelper {
            enableAxisLines: true
            enableXYGrid: true
            enableXZGrid: true
            enableYZGrid: true
            gridColor: "black"
            gridOpacity: 0.1
        }
        
        // Light
        DirectionalLight {
            id: directionalLight
            x: 0
            y: 0
            z: 1000
            eulerRotation.x: 30
            eulerRotation.y: 70
            eulerRotation.z: 0
            brightness: 1.0
            ambientColor: Qt.rgba(0.1, 0.1, 0.1, 1.0)
            castsShadow: true
        }
        
        // Link1 Model
        Node {
            id: link1_node
            eulerRotation.z: joint1Angle  // 将第一个滑条的值绑定到节点的Z轴旋转
            scale: Qt.vector3d(100, 100, 100)  // 将模型放大100倍
            
            // 材质定义
            PrincipledMaterial {
                id: link1_material
                baseColor: "#808080"
                metalness: 0.3
                roughness: 0.4
            }
            
            // Mesh模型
            Model {
                id: base_model
                source: "model/meshes/base_mesh.mesh"
                materials: [link1_material]
            }
            
            Model {
                id: base_1_model
                source: "model/meshes/base__1__mesh.mesh"
                materials: [link1_material]
            }
        }
    }

    // Camera
    PerspectiveCamera{
        id: camera
        x: 500  // 调整相机位置，使其能够看到放大后的模型
        y: 1250
        z: 4000
        eulerRotation.x: 0
        eulerRotation.y: 0
        eulerRotation.z: 0

        clipNear: 0
        clipFar: 30000  // 增加远裁剪面距离，确保能看到整个放大后的模型
        fieldOfView: 60
        fieldOfViewOrientation: Camera.Horizontal
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
                let deltaX = mouse.x - middleCx
                let deltaY = mouse.y - middleCy
                camera.eulerRotation.x = camera.eulerRotation.x - deltaY
                camera.eulerRotation.y = camera.eulerRotation.y - deltaX
                middleCx = mouse.x
                middleCy = mouse.y
            }
            if (rightPress) {
                let deltaX = mouse.x - rightCx
                let deltaY = mouse.y - rightCy
                camera.x = camera.x - (0.000027 * deltaX * camera.z * camera.fieldOfView)
                camera.y = camera.y + (0.000027 * deltaY * camera.z * camera.fieldOfView)
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
            if (wheel.angleDelta.y > 0) {
                // 放大时限制最小距离
                camera.z = Math.max(100, camera.z * 0.9)
            } else {
                // 缩小时限制最大距离
                camera.z = Math.min(3000, camera.z * 1.1)
            }
        }
    }

}