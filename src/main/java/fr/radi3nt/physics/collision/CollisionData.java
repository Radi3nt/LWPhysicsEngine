package fr.radi3nt.physics.collision;

import fr.radi3nt.physics.collision.shape.CollisionShape;
import fr.radi3nt.physics.collision.shape.pre.PreCollisionShape;
import fr.radi3nt.physics.collision.shape.provider.CollisionShapeProvider;
import fr.radi3nt.physics.collision.shape.provider.SetCollisionShapeProvider;
import fr.radi3nt.physics.core.TransformedObject;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.Optional;

public class CollisionData implements CollisionShapeGroup {

    private final CollisionShapeProvider collisionShapeProvider;
    private final PreCollisionShape preCollisionShape;

    public CollisionData(CollisionShape collisionShape) {
        this.collisionShapeProvider = new SetCollisionShapeProvider(collisionShape);
        preCollisionShape = collisionShape.getPreCollisionShape();
    }

    public CollisionData(CollisionShape collisionShape, PreCollisionShape preCollisionShape) {
        this.collisionShapeProvider = new SetCollisionShapeProvider(collisionShape);
        this.preCollisionShape = preCollisionShape;
    }

    public CollisionData(CollisionShapeProvider collisionShapeProvider, PreCollisionShape preCollisionShape) {
        this.collisionShapeProvider = collisionShapeProvider;
        this.preCollisionShape = preCollisionShape;
    }

    public CollisionShape[] getCollisionShape(RigidBody other) {
        return collisionShapeProvider.getCollisionShape(other);
    }

    public CollisionShapeProvider getCollisionShapeProvider() {
        return collisionShapeProvider;
    }

    public PreCollisionShape getPreCollisionShape() {
        return preCollisionShape;
    }
}
