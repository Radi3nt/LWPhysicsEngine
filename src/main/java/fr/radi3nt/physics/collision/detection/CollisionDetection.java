package fr.radi3nt.physics.collision.detection;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.contact.cache.ContactPairCache;
import fr.radi3nt.physics.collision.contact.cache.ContactPairCacheProvider;
import fr.radi3nt.physics.collision.contact.cache.PersistentManifoldCache;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;
import fr.radi3nt.physics.collision.detection.narrow.dispacher.CollisionDispatcher;

import java.util.ArrayList;
import java.util.Collection;

public class CollisionDetection {

    private final ContactPairCacheProvider cacheProvider;
    private final PersistentManifoldCache manifoldCache;
    private final CollisionDispatcher collisionDispatcher;

    public CollisionDetection(ContactPairCacheProvider cacheProvider, PersistentManifoldCache manifoldCache, CollisionDispatcher collisionDispatcher) {
        this.cacheProvider = cacheProvider;
        this.manifoldCache = manifoldCache;
        this.collisionDispatcher = collisionDispatcher;
    }

    public Collection<PersistentManifold> process(long currentStep) {
        ContactPairCache cache = cacheProvider.newFilledCache();

        Collection<PersistentManifold> collidingManifolds = new ArrayList<>();
        for (GeneratedContactPair contactPair : cache) {
            PersistentManifold manifold = collisionDispatcher.dispatch(manifoldCache, contactPair, currentStep);
            if (manifold==null)
                continue;
            collidingManifolds.add(manifold);
        }

        return collidingManifolds;
    }
}
