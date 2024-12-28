package fr.radi3nt.physics.collision.contact;

import fr.radi3nt.physics.collision.shape.shapes.CollisionShape;
import fr.radi3nt.physics.core.TransformedObject;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.Objects;

public class GeneratedContactPair<T extends TransformedObject> {

    public final T objectA;
    public final T objectB;

    public final CollisionShape shapeA;
    public final CollisionShape shapeB;

    public GeneratedContactPair(T objectA, CollisionShape shapeA, T objectB, CollisionShape shapeB) {
        this.objectA = objectA;
        this.shapeA = shapeA;
        this.objectB = objectB;
        this.shapeB = shapeB;
    }

    public static ContactKeyPair toPair(GeneratedContactPair<RigidBody> pair) {
        return new ContactKeyPair(pair.objectA.getRigidBodyId(), pair.objectB.getRigidBodyId(), pair.shapeA, pair.shapeB);
    }

    @Override
    public final boolean equals(Object o) {
        if (!(o instanceof GeneratedContactPair)) return false;

        GeneratedContactPair<?> that = (GeneratedContactPair<?>) o;
        return Objects.equals(objectA, that.objectA) && Objects.equals(objectB, that.objectB) && Objects.equals(shapeA, that.shapeA) && Objects.equals(shapeB, that.shapeB);
    }

    @Override
    public int hashCode() {
        int result = Objects.hashCode(objectA);
        result = 31 * result + Objects.hashCode(objectB);
        result = 31 * result + Objects.hashCode(shapeA);
        result = 31 * result + Objects.hashCode(shapeB);
        return result;
    }
}
