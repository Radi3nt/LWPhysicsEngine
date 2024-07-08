package fr.radi3nt.physics.collision.contact;

import fr.radi3nt.physics.collision.shape.CollisionShapeProvider;
import fr.radi3nt.physics.collision.shape.shapes.CollisionShape;

import java.util.Objects;

public class ContactKeyPair {

    private final int bodyA;
    private final int bodyB;

    public final CollisionShape shapeA;
    public final CollisionShape shapeB;

    public ContactKeyPair(int bodyA, int bodyB, CollisionShape shapeA, CollisionShape shapeB) {
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.shapeA = shapeA;
        this.shapeB = shapeB;
    }

    @Override
    public final boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof ContactKeyPair)) return false;

        ContactKeyPair that = (ContactKeyPair) o;
        return bodyA == that.bodyA && bodyB == that.bodyB && Objects.equals(shapeA, that.shapeA) && Objects.equals(shapeB, that.shapeB);
    }

    @Override
    public int hashCode() {
        int result = bodyA;
        result = 31 * result + bodyB;
        result = 31 * result + Objects.hashCode(shapeA);
        result = 31 * result + Objects.hashCode(shapeB);
        return result;
    }
}
