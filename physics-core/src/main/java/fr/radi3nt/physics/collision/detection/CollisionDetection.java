package fr.radi3nt.physics.collision.detection;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.contact.cache.ContactPairCache;
import fr.radi3nt.physics.collision.contact.cache.ContactPairCacheProvider;
import fr.radi3nt.physics.collision.contact.cache.PersistentManifoldCache;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;
import fr.radi3nt.physics.collision.detection.narrow.dispacher.CollisionDispatcher;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.ArrayList;
import java.util.Collection;
import java.util.concurrent.Callable;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;

public class CollisionDetection {

    private static final int BATCH_SIZE = 20;

    private final ContactPairCacheProvider cacheProvider;
    private final PersistentManifoldCache manifoldCache;
    private final CollisionDispatcher collisionDispatcher;

    private final ExecutorService service;

    public CollisionDetection(ContactPairCacheProvider cacheProvider, PersistentManifoldCache manifoldCache, CollisionDispatcher collisionDispatcher, ExecutorService service) {
        this.cacheProvider = cacheProvider;
        this.manifoldCache = manifoldCache;
        this.collisionDispatcher = collisionDispatcher;
        this.service = service;
    }

    public Collection<PersistentManifold> process(long currentStep) {
        ContactPairCache<RigidBody> cache = cacheProvider.newFilledCache();

        Collection<Callable<Object>> callables = new ArrayList<>();
        Collection<PersistentManifold> collidingManifolds = ConcurrentHashMap.newKeySet();

        Collection<GeneratedContactPair<RigidBody>> subset = new ArrayList<>();
        for (GeneratedContactPair<RigidBody> contactPair : cache) {
            subset.add(contactPair);
            if (subset.size()>=BATCH_SIZE) {
                submit(currentStep, callables, subset, collidingManifolds);
                subset = new ArrayList<>();
            }
        }

        if (!subset.isEmpty()) {
            submit(currentStep, callables, subset, collidingManifolds);
        }

        try {
            service.invokeAll(callables);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        return collidingManifolds;
    }

    private void submit(long currentStep, Collection<Callable<Object>> callables, Collection<GeneratedContactPair<RigidBody>> finalSubset, Collection<PersistentManifold> collidingManifolds) {
        callables.add(() -> {
            for (GeneratedContactPair<RigidBody> pair : finalSubset) {
                PersistentManifold manifold = collisionDispatcher.dispatch(manifoldCache, pair, currentStep);
                if (manifold==null)
                    continue;
                collidingManifolds.add(manifold);
            }
            return null;
        });
    }
}
