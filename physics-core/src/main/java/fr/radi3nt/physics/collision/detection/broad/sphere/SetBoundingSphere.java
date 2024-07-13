package fr.radi3nt.physics.collision.detection.broad.sphere;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.core.TransformedObject;

public class SetBoundingSphere implements BoundingSphere {

    private final Vector3f position;
    private final float radius;

    public SetBoundingSphere(Vector3f position, float radius) {
        this.position = position;
        this.radius = radius;
    }

    public static SetBoundingSphere from(TransformedObject object, Vector3f position, float radius) {
        return new SetBoundingSphere(object.getPosition().duplicate().add(position), radius);
    }

    public Vector3f getPosition() {
        return position;
    }

    public float getRadius() {
        return radius;
    }
}
