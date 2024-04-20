package fr.radi3nt.physics.collision.shape.sat;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.shape.pre.PreCollisionShape;
import fr.radi3nt.physics.collision.shape.pre.SatPreCollisionShape;
import fr.radi3nt.physics.collision.shape.sat.shape.SatShapeObject;

import java.util.Optional;

public class SetSatCollisionShape implements SatCollisionShape {

    private final SatShapeObject satShapeObject;
    private final PreCollisionShape collisionShape;

    public SetSatCollisionShape(SatShapeObject satShapeObject, Vector3f offset) {
        this.satShapeObject = satShapeObject;
        collisionShape = new SatPreCollisionShape(satShapeObject, offset);
    }

    public SetSatCollisionShape(SatShapeObject satShapeObject) {
        this.satShapeObject = satShapeObject;
        collisionShape = new SatPreCollisionShape(satShapeObject, new SimpleVector3f());
    }

    @Override
    public PreCollisionShape getPreCollisionShape() {
        return collisionShape;
    }

    @Override
    public SatShapeObject getShape() {
        return satShapeObject;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof SetSatCollisionShape)) return false;

        SetSatCollisionShape that = (SetSatCollisionShape) o;

        return satShapeObject.equals(that.satShapeObject);
    }

    @Override
    public int hashCode() {
        return satShapeObject.hashCode();
    }
}
