package fr.radi3nt.physics.collision.shape.provider;

import fr.radi3nt.physics.collision.shape.CollisionShapeProvider;
import fr.radi3nt.physics.collision.shape.DuoCollisionShape;
import fr.radi3nt.physics.collision.detection.broad.pre.PreCollisionShape;
import fr.radi3nt.physics.collision.shape.shapes.CollisionShape;
import fr.radi3nt.physics.core.state.RigidBody;

public class SetCollisionShapeProvider implements CollisionShapeProvider {

    private final DuoCollisionShape[] collisionShapes;
    private final PreCollisionShape preCollisionShape;

    public SetCollisionShapeProvider(PreCollisionShape preCollisionShape, DuoCollisionShape... collisionShapes) {
        this.preCollisionShape = preCollisionShape;
        this.collisionShapes = collisionShapes;
    }

    public static SetCollisionShapeProvider from(CollisionShape collisionShape) {
        if (collisionShape instanceof PreCollisionShape) {
            return new SetCollisionShapeProvider((PreCollisionShape) collisionShape, new DuoCollisionShape(collisionShape, null));
        } else {
            return new SetCollisionShapeProvider(null, new DuoCollisionShape(collisionShape, null));
        }
    }

    @Override
    public DuoCollisionShape[] getCollisionShapes(RigidBody other) {
        return collisionShapes;
    }

    @Override
    public boolean isEmpty() {
        return collisionShapes.length==0;
    }

    public DuoCollisionShape[] getCollisionShapes() {
        return collisionShapes;
    }

    @Override
    public PreCollisionShape getPreCollisionShape() {
        return preCollisionShape;
    }
}
