package fr.radi3nt.physics.collision.detection.generators.generator;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.detection.broad.pre.PreCollisionPair;
import fr.radi3nt.physics.collision.shape.DuoCollisionShape;
import fr.radi3nt.physics.collision.shape.pre.PreCollisionData;
import fr.radi3nt.physics.collision.shape.shapes.CollisionShape;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.Comparator;
import java.util.List;
import java.util.function.Predicate;

public class BroadphaseOrderingPairGenerator implements PairGenerator {

    private static final Comparator<RigidBody> rigidBodyComparator = Comparator.comparingInt(RigidBody::getRigidBodyId);
    private final Predicate<PreCollisionPair> canSkipCollision;

    public BroadphaseOrderingPairGenerator(Predicate<PreCollisionPair> canSkipCollision) {
        this.canSkipCollision = canSkipCollision;
    }

    @Override
    public void pair(List<GeneratedContactPair> pairList, RigidBody a, RigidBody b, PreCollisionData aData, PreCollisionData bData) {
        if ((a.isStatic() && b.isStatic()) || a.getCollisionData().cannotCollide(b) || b.getCollisionData().cannotCollide(a))
            return;

        PreCollisionPair pair = new PreCollisionPair(a, b);

        if (aData!=null && bData!=null && canSkipCollision.test(pair.from(aData, bData)))
            return;

        DuoCollisionShape[] objectsA = a.getCollisionData().getCollisionShape(b);
        DuoCollisionShape[] objectsB = b.getCollisionData().getCollisionShape(a);

        for (DuoCollisionShape objA : objectsA) {
            for (DuoCollisionShape objB : objectsB) {
                if (PreCollisionPair.isValid(objA.preCollisionShape, objB.preCollisionShape) && canSkipCollision.test(pair.from(objA, objB)))
                    continue;
                pairList.add(getContactPairs(a, b, objA.collisionShape, objB.collisionShape));
            }
        }
    }

    private static GeneratedContactPair getContactPairs(RigidBody rigidBodyA, RigidBody rigidBodyB, CollisionShape a, CollisionShape b) {
        int compare = rigidBodyComparator.compare(rigidBodyA, rigidBodyB);
        if (compare < 0) {
            CollisionShape cacheShape = a;
            RigidBody cacheRigidBody = rigidBodyA;
            a = b;
            rigidBodyA = rigidBodyB;
            b = cacheShape;
            rigidBodyB = cacheRigidBody;
        }

        return new GeneratedContactPair(rigidBodyA, a, rigidBodyB, b);
    }
}
