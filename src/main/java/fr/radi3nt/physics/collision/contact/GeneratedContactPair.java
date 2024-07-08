package fr.radi3nt.physics.collision.contact;

import fr.radi3nt.physics.collision.shape.shapes.CollisionShape;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.Objects;

public class GeneratedContactPair {

    public final RigidBody objectA;
    public final RigidBody objectB;

    public final CollisionShape shapeA;
    public final CollisionShape shapeB;

    public GeneratedContactPair(RigidBody objectA, CollisionShape shapeA, RigidBody objectB, CollisionShape shapeB) {
        this.objectA = objectA;
        this.shapeA = shapeA;
        this.objectB = objectB;
        this.shapeB = shapeB;
    }

    public ContactKeyPair toPair() {
        return new ContactKeyPair(objectA.getRigidBodyId(), objectB.getRigidBodyId(), shapeA, shapeB);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof GeneratedContactPair)) return false;

        GeneratedContactPair that = (GeneratedContactPair) o;

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
