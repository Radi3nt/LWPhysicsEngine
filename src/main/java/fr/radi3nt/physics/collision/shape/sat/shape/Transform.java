package fr.radi3nt.physics.collision.shape.sat.shape;

import fr.radi3nt.maths.components.advanced.quaternions.ComponentsQuaternion;
import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;

public class Transform {

    private final Vector3f position;
    private final Quaternion rotation;
    private final Vector3f size;

    public Transform(Vector3f position, Quaternion quaternion, Vector3f size) {
        this.position = position;
        rotation = quaternion;
        this.size = size;
    }

    public static Transform zero(Vector3f size) {
        return new Transform(new SimpleVector3f(), ComponentsQuaternion.zero(), size);
    }

    public Vector3f getPosition() {
        return position;
    }

    public Quaternion getRotation() {
        return rotation;
    }

    public Vector3f getSize() {
        return size;
    }

    public Vector3f transformNormal(Vector3f transformed) {
        rotation.transform(transformed);
        return transformed;
    }

    public Vector3f transform(Vector3f transformed) {
        transformed.mul(size);
        transformNormal(transformed);
        transformed.add(position);
        return transformed;
    }
}
