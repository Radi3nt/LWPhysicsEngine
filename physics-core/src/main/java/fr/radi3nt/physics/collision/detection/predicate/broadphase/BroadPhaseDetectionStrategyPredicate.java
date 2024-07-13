package fr.radi3nt.physics.collision.detection.predicate.broadphase;

import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.detection.broad.BroadPhaseDetectionStrategy;
import fr.radi3nt.physics.collision.detection.broad.pre.PreCollisionPair;
import fr.radi3nt.physics.collision.detection.broad.pre.PreCollisionShape;
import fr.radi3nt.physics.core.TransformedObject;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.function.Predicate;

public class BroadPhaseDetectionStrategyPredicate implements Predicate<RigidBody> {

    private final BroadPhaseDetectionStrategy strategy;
    private RigidBody otherRigidBody;

    private final PreCollisionPair pair;

    public BroadPhaseDetectionStrategyPredicate(BroadPhaseDetectionStrategy strategy, TransformedObject transformedObject, PreCollisionShape preCollisionShape) {
        this.strategy = strategy;
        TransformedObject otherTransformedObject = new TransformedObject() {
            @Override
            public Vector3f getPosition() {
                return otherRigidBody.getPosition();
            }

            @Override
            public Quaternion getRotation() {
                return otherRigidBody.getRotation();
            }
        };
        pair = new PreCollisionPair(transformedObject, otherTransformedObject);
        pair.setShapeA(preCollisionShape);
    }

    @Override
    public boolean test(RigidBody rigidBody) {
        PreCollisionShape preCollisionShape = rigidBody.getCollisionData().getPreCollisionShape();
        if (preCollisionShape==null)
            return false;
        otherRigidBody = rigidBody;
        pair.setShapeB(preCollisionShape);
        return !strategy.canSkip(pair);
    }
}
