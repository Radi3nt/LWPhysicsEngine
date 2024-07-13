package fr.radi3nt.physics.collision.shape.provider;

import fr.radi3nt.physics.collision.shape.CollisionShapeProvider;
import fr.radi3nt.physics.collision.shape.DuoCollisionShape;
import fr.radi3nt.physics.collision.detection.broad.pre.PreCollisionShape;
import fr.radi3nt.physics.core.state.RigidBody;

public class SetCollisionShapeProvider implements CollisionShapeProvider {

    private final DuoCollisionShape[] collisionShapes;
    private final PreCollisionShape preCollisionShape;

    public SetCollisionShapeProvider(PreCollisionShape preCollisionShape, DuoCollisionShape... collisionShapes) {
        this.preCollisionShape = preCollisionShape;
        this.collisionShapes = collisionShapes;
    }

    @Override
    public DuoCollisionShape[] getCollisionShapes(RigidBody other) {
        return collisionShapes;
    }

    public DuoCollisionShape[] getCollisionShapes() {
        return collisionShapes;
    }

    @Override
    public PreCollisionShape getPreCollisionShape() {
        return preCollisionShape;
    }
}
