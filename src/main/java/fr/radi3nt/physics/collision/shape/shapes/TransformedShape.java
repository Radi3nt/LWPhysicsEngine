package fr.radi3nt.physics.collision.shape.shapes;

import fr.radi3nt.maths.components.advanced.quaternions.ComponentsQuaternion;
import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;

import java.util.Objects;

public abstract class TransformedShape implements CollisionShape {

    protected final Vector3f offset;
    protected final Quaternion rotation;

    public TransformedShape(Vector3f offset, Quaternion rotation) {
        this.offset = offset;
        this.rotation = rotation;
    }

    public TransformedShape() {
        this(new SimpleVector3f(), ComponentsQuaternion.zero());
    }

    public Vector3f getOffset() {
        return offset;
    }

    public Quaternion getRotation() {
        return rotation;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof TransformedShape)) return false;

        TransformedShape that = (TransformedShape) o;
        return Objects.equals(offset, that.offset) && Objects.equals(rotation, that.rotation);
    }

    @Override
    public int hashCode() {
        int result = Objects.hashCode(offset);
        result = 31 * result + Objects.hashCode(rotation);
        return result;
    }
}
