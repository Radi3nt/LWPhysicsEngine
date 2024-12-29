package fr.radi3nt.physics.collision.shape;

import fr.radi3nt.physics.collision.detection.broad.pre.PreCollisionHolder;
import fr.radi3nt.physics.collision.detection.broad.pre.PreCollisionShape;
import fr.radi3nt.physics.collision.shape.shapes.CollisionShape;

import java.util.Objects;

public class DuoCollisionShape implements PreCollisionHolder {

    public final CollisionShape collisionShape;
    public final PreCollisionShape preCollisionShape;

    public DuoCollisionShape(CollisionShape collisionShape, PreCollisionShape preCollisionShape) {
        this.collisionShape = collisionShape;
        this.preCollisionShape = preCollisionShape;
    }

    public static DuoCollisionShape from(CollisionShape shape) {
        return new DuoCollisionShape(shape, shape instanceof PreCollisionShape ? (PreCollisionShape) shape : null);
    }

    @Override
    public PreCollisionShape getPreCollisionShape() {
        return preCollisionShape;
    }

    @Override
    public final boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof DuoCollisionShape)) return false;

        DuoCollisionShape that = (DuoCollisionShape) o;
        return Objects.equals(collisionShape, that.collisionShape) && Objects.equals(preCollisionShape, that.preCollisionShape);
    }

    @Override
    public int hashCode() {
        int result = Objects.hashCode(collisionShape);
        result = 31 * result + Objects.hashCode(preCollisionShape);
        return result;
    }

    @Override
    public String toString() {
        return "DuoCollisionShape{" +
                "collisionShape=" + collisionShape +
                ", preCollisionShape=" + preCollisionShape +
                '}';
    }
}
