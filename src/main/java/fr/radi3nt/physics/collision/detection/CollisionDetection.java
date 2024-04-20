package fr.radi3nt.physics.collision.detection;

import fr.radi3nt.physics.collision.contact.ContactPair;
import fr.radi3nt.physics.collision.contact.cache.ContactPairCache;
import fr.radi3nt.physics.collision.contact.cache.ContactPairCacheProvider;
import fr.radi3nt.physics.collision.contact.cache.PersistentManifoldCache;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;
import fr.radi3nt.physics.collision.detection.broad.BroadPhaseDetectionStrategy;
import fr.radi3nt.physics.collision.detection.narrow.dispacher.CollisionDispatcher;
import fr.radi3nt.physics.collision.response.CollisionContactSolver;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.ArrayList;
import java.util.Collection;
import java.util.ListIterator;
import java.util.Optional;

public class CollisionDetection {

    private final ContactPairCacheProvider cacheProvider;
    private final PersistentManifoldCache manifoldCache;
    private final CollisionDispatcher collisionDispatcher;
    private final CollisionContactSolver collisionContactSolver;

    public CollisionDetection(ContactPairCacheProvider cacheProvider, PersistentManifoldCache manifoldCache, CollisionDispatcher collisionDispatcher, CollisionContactSolver collisionContactSolver) {
        this.cacheProvider = cacheProvider;
        this.manifoldCache = manifoldCache;
        this.collisionDispatcher = collisionDispatcher;
        this.collisionContactSolver = collisionContactSolver;
    }

    public void process(float dt) {
        ContactPairCache cache = cacheProvider.newFilledCache();
        //System.out.println("size: " + cache.size());
        Collection<PersistentManifold> collidingManifolds = new ArrayList<>();
        for (ContactPair contactPair : cache) {
            Optional<PersistentManifold> manifold = collisionDispatcher.dispatch(manifoldCache, contactPair);
            manifold.ifPresent(collidingManifolds::add);
        }

        collisionContactSolver.solve(collidingManifolds, dt);
    }
}
