package fr.radi3nt.physics.collision.detection.gen.generator;

import fr.radi3nt.physics.collision.contact.ContactPair;
import fr.radi3nt.physics.collision.detection.broad.aabb.CachingAABB;
import fr.radi3nt.physics.collision.shape.CollisionShape;
import fr.radi3nt.physics.collision.shape.pre.PreCollisionPair;
import fr.radi3nt.physics.collision.shape.pre.PreCollisionShape;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BiPredicate;
import java.util.function.Predicate;

public class BroadphaseOrderingPairGenerator implements PairGenerator {

    private static final Comparator<RigidBody> rigidBodyComparator = Comparator.comparingInt(RigidBody::getRigidBodyId);
    private final Predicate<PreCollisionPair> canSkipCollision;

    public BroadphaseOrderingPairGenerator(Predicate<PreCollisionPair> canSkipCollision) {
        this.canSkipCollision = canSkipCollision;
    }

    @Override
    public void pair(List<ContactPair> pairList, RigidBody a, RigidBody b) {
        if ((a.getSleepingData().isSleeping() || a.getDynamicsData().getBodyProperties().inverseMass==0) && (b.getSleepingData().isSleeping()  || b.getDynamicsData().getBodyProperties().inverseMass==0))
            return;

        PreCollisionPair pair = new PreCollisionPair(a, b);

        CollisionShape[] objectsA = a.getCollisionData().getCollisionShape(b);
        CollisionShape[] objectsB = b.getCollisionData().getCollisionShape(a);

        for (CollisionShape objA : objectsA) {
            for (CollisionShape objB : objectsB) {
                if (PreCollisionPair.isValid(objA, objB) && canSkipCollision.test(pair.from(objA, objB)))
                    continue;
                pairList.add(getContactPairs(a, b, objA, objB));
            }
        }
    }

    private static ContactPair getContactPairs(RigidBody rigidBodyA, RigidBody rigidBodyB, CollisionShape a, CollisionShape b) {
        int compare = rigidBodyComparator.compare(rigidBodyA, rigidBodyB);
        if (compare < 0) {
            CollisionShape cacheShape = a;
            RigidBody cacheRigidBody = rigidBodyA;
            a = b;
            rigidBodyA = rigidBodyB;
            b = cacheShape;
            rigidBodyB = cacheRigidBody;
        }

        return new ContactPair(rigidBodyA, a, rigidBodyB, b);
    }
}
