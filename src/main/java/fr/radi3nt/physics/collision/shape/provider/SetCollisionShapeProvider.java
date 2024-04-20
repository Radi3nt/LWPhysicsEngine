package fr.radi3nt.physics.collision.shape.provider;

import fr.radi3nt.physics.collision.shape.CollisionShape;
import fr.radi3nt.physics.core.TransformedObject;
import fr.radi3nt.physics.core.state.RigidBody;

public class SetCollisionShapeProvider implements CollisionShapeProvider, IndependentCollisionShapeProvider {

    private final CollisionShape[] collisionShapes;

    public SetCollisionShapeProvider(CollisionShape... collisionShapes) {
        this.collisionShapes = collisionShapes;
    }

    @Override
    public CollisionShape[] getCollisionShape(RigidBody other) {
        return collisionShapes;
    }

    @Override
    public CollisionShape[] getCollisionShape() {
        return collisionShapes;
    }
}
