import QtQuick
import QtQuick3D

Node {
    id: node

    // Resources
    PrincipledMaterial {
        id: opaque_255_255_255__material
        objectName: "Opaque(255,255,255)"
        baseColor: "#ff999999"
        indexOfRefraction: 1
    }

    // Nodes:
    Node {
        id: link2_obj
        objectName: "link2.obj"
        Model {
            id: __1
            objectName: "实体1"
            source: "meshes/__1_mesh.mesh"
            materials: [
                opaque_255_255_255__material
            ]
        }
        Model {
            id: __2
            objectName: "实体2"
            source: "meshes/__2_mesh.mesh"
            materials: [
                opaque_255_255_255__material
            ]
        }
    }

    // Animations:
}
