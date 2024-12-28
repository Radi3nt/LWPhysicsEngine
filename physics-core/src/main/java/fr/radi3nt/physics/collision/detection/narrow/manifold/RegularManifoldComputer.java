package fr.radi3nt.physics.collision.detection.narrow.manifold;

import fr.radi3nt.physics.collision.CollisionData;
import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.contact.cache.PersistentManifoldCache;
import fr.radi3nt.physics.collision.contact.manifold.ManifoldPoint;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.Collection;

public class RegularManifoldComputer {

    public PersistentManifold compute(PersistentManifoldCache manifoldCache, GeneratedContactPair<RigidBody> pair, Collection<ManifoldPoint> pairResults, long currentStep) {

        for (CollisionData.PointFilter collectionConsumer : pair.objectA.getCollisionData().getPointsFilter()) {
            collectionConsumer.filter(pairResults, pair.objectB, currentStep);
        }
        for (CollisionData.PointFilter collectionConsumer : pair.objectB.getCollisionData().getPointsFilter()) {
            collectionConsumer.filter(pairResults, pair.objectA, currentStep);
        }

        if (pairResults.isEmpty())
            return null;

        PersistentManifold result = manifoldCache.getCachedManifold(pair);

        if (result==null) {
            if (!pairResults.isEmpty()) {
                PersistentManifold current = manifoldCache.newManifold(pair);
                updatePoints(pair, pairResults, currentStep, current);
                return current;
            }
        } else {
            updatePoints(pair, pairResults, currentStep, result);
        }

        return result;
    }

    private static void updatePoints(GeneratedContactPair<RigidBody> pair, Collection<ManifoldPoint> pairResults, long currentStep, PersistentManifold result) {
        result.replaceManifoldPoints(pairResults);
        result.refresh(pair.objectA, pair.objectB, currentStep);
        result.addManifoldPoints(pairResults);
    }
}
