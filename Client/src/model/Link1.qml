import QtQuick
import QtQuick3D

Node {
    id: node

    // Resources
    PrincipledMaterial {
        id: opaque_191_191_191__material
        objectName: "Opaque(191,191,191)"
        baseColor: "#ffbfbfbf"
        indexOfRefraction: 1
    }

    // Nodes:
    Node {
        id: link1_obj
        objectName: "link1.obj"
        Model {
            id: base
            objectName: "base"
            source: "meshes/base_mesh.mesh"
            materials: [
                opaque_191_191_191__material
            ]
        }
        Model {
            id: base__1_
            objectName: "base (1)"
            source: "meshes/base__1__mesh.mesh"
            materials: [
                opaque_191_191_191__material
            ]
        }
    }

    // Animations:
}
