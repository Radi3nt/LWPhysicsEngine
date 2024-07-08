package fr.radi3nt.physics.collision.detection.narrow.manifold;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.contact.cache.PersistentManifoldCache;
import fr.radi3nt.physics.collision.contact.manifold.ManifoldPoint;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;

import java.util.Collection;

public class RegularManifoldComputer {

    public PersistentManifold compute(PersistentManifoldCache manifoldCache, GeneratedContactPair pair, Collection<ManifoldPoint> pairResults, long currentStep) {
        PersistentManifold result = manifoldCache.getCachedManifold(pair);

        if (result==null) {
            if (!pairResults.isEmpty()) {
                PersistentManifold current = manifoldCache.newManifold(pair);
                updatePoints(pair, pairResults, currentStep, current);
                return current;
            }
        } else {
            updatePoints(pair, pairResults, currentStep, result);

            if (result.isEmpty()) {
                manifoldCache.releaseManifold(pair);
                return null;
            }
        }

        return result;
    }

    private static void updatePoints(GeneratedContactPair pair, Collection<ManifoldPoint> pairResults, long currentStep, PersistentManifold result) {
        result.replaceManifoldPoints(pairResults);
        result.refresh(pair.objectA, pair.objectB, currentStep);
        result.addManifoldPoints(pairResults);
    }
}
