package fr.radi3nt.physics.collision.contact;

import fr.radi3nt.physics.collision.CollisionShapeGroup;
import fr.radi3nt.physics.collision.shape.CollisionShape;
import fr.radi3nt.physics.core.TransformedObject;

import java.util.Objects;

public class ContactPair {

    public final TransformedObject objectA;
    public final CollisionShape shapeA;
    public final TransformedObject objectB;
    public final CollisionShape shapeB;

    public ContactPair(TransformedObject objectA, CollisionShape shapeA, TransformedObject objectB, CollisionShape shapeB) {
        this.objectA = objectA;
        this.shapeA = shapeA;
        this.objectB = objectB;
        this.shapeB = shapeB;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof ContactPair)) return false;

        ContactPair that = (ContactPair) o;

        if (!Objects.equals(objectA, that.objectA)) return false;
        if (!Objects.equals(shapeA, that.shapeA)) return false;
        if (!Objects.equals(objectB, that.objectB)) return false;
        return Objects.equals(shapeB, that.shapeB);
    }

    @Override
    public int hashCode() {
        int result = objectA != null ? objectA.hashCode() : 0;
        result = 31 * result + (shapeA != null ? shapeA.hashCode() : 0);
        result = 31 * result + (objectB != null ? objectB.hashCode() : 0);
        result = 31 * result + (shapeB != null ? shapeB.hashCode() : 0);
        return result;
    }
}
