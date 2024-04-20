package fr.radi3nt.physics.collision.detection.narrow.sat;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.contact.ContactPair;
import fr.radi3nt.physics.collision.contact.cache.PersistentManifoldCache;
import fr.radi3nt.physics.collision.contact.manifold.ManifoldPoint;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;
import fr.radi3nt.physics.collision.detection.narrow.sat.computer.CollisionResult;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;

public class SatManifoldComputer {

    private static final float EPSILON = 1e-1f;
    private static final float RELATIVE_EPSILON = 2e-1f;
    private final Vector3f cache = new SimpleVector3f();

    public Optional<PersistentManifold> compute(PersistentManifoldCache manifoldCache, ContactPair pair, CollisionResult collisionResult) {
        Optional<PersistentManifold> result = manifoldCache.getCachedManifold(pair);
        Vector3f directedNormal = collisionResult.getWorldSpaceDirectedNormal();

        List<ManifoldPoint> pairResults = createContactPoints(pair, collisionResult);

        if (!result.isPresent()) {
            if (!pairResults.isEmpty()) {
                PersistentManifold current = manifoldCache.newManifold(pair);
                updateManifoldInformation(current, directedNormal, pair, collisionResult.getOverlap());
                current.getManifoldPoints().clear();
                current.getManifoldPoints().addAll(pairResults);
                return Optional.of(current);
            }
        } else {
            Collection<ManifoldPoint> toRemove = new ArrayList<>();
            for (ManifoldPoint manifoldPoint : result.get().getManifoldPoints()) {
                manifoldPoint.enabled = false;

                for (ManifoldPoint pairResult : pairResults) {
                    if (manifoldPoint.index.equals(pairResult.index)) {
                        pairResult.cachedContactLambda = manifoldPoint.cachedContactLambda;
                        pairResult.cachedFrictionLambda = manifoldPoint.cachedFrictionLambda;
                        toRemove.add(manifoldPoint);
                        pairResult.enabled = true;
                        break;
                    }
                }
            }

            updateManifoldInformation(result.get(), directedNormal, pair, collisionResult.getOverlap());
            //result.get().getManifoldPoints().clear();
            result.get().getManifoldPoints().removeAll(toRemove);
            result.get().getManifoldPoints().addAll(pairResults);

            if (result.get().isEmpty()) {
                manifoldCache.releaseManifold(pair);
                return Optional.empty();
            }
        }

        //todo notify objects of collisions or something? maybe not in here but yeah

        return result;
    }

    private static void updateManifoldInformation(PersistentManifold current, Vector3f normal, ContactPair pair, float currentOverlap) {
        current.setDistanceThreshold(1e-1f);
        current.setNormal(normal);
        current.setPair(pair.objectA, pair.objectB);
    }

    private List<ManifoldPoint> createContactPoints(ContactPair pair, CollisionResult collisionResult) {
        List<ManifoldPoint> pairResults = collisionResult.createPoints();

        removeDuplicatedContactsPoints(pairResults, pair, collisionResult.getWorldSpaceDirectedNormal());
        return pairResults;
    }

    private void removeDuplicatedContactsPoints(List<ManifoldPoint> pairResults, ContactPair pair, Vector3f directedNormal) {
        for (ManifoldPoint pairResult : pairResults) {
            pairResult.enabled = true;
            pairResult.refresh(pair.objectA, pair.objectB, directedNormal);
        }

        for (int i = 0; i < pairResults.size(); i++) {
            ManifoldPoint manifold1 = pairResults.get(i);

            Vector3f pointA1 = manifold1.worldSpaceA;
            Vector3f pointB1 = manifold1.worldSpaceB;

            for (int j = 0; j < pairResults.size(); j++) {
                if (j<=i)
                    continue;

                ManifoldPoint manifold2 = pairResults.get(j);

                Vector3f pointA2 = manifold2.worldSpaceA;
                Vector3f pointB2 = manifold2.worldSpaceB;

                float epsilon = EPSILON;

                cache.copy(pointA1);
                if (cache.sub(pointA2).lengthSquared() < epsilon) {
                    float projected1 = directedNormal.dot(pointB1);
                    float projected2 = directedNormal.dot(pointB2);
                    if (projected1>projected2) {
                        pairResults.remove(j);
                    } else {
                        pairResults.remove(i);
                        i--;
                        break;
                    }
                    j--;
                    continue;
                }
                cache.copy(pointB1);
                if (cache.sub(pointB2).lengthSquared() < epsilon) {
                    float projected1 = directedNormal.dot(pointA1);
                    float projected2 = directedNormal.dot(pointA2);
                    if (projected1>projected2) {
                        pairResults.remove(j);
                    } else {
                        pairResults.remove(i);
                        i--;
                        break;
                    }
                    j--;
                    continue;
                }
                cache.copy(pointA1);
                if (cache.sub(pointB2).lengthSquared() < epsilon) {
                    float projected1 = directedNormal.dot(pointB1);
                    float projected2 = directedNormal.dot(pointA2);
                    if (projected1>projected2) {
                        pairResults.remove(j);
                    } else {
                        pairResults.remove(i);
                        i--;
                        break;
                    }
                    j--;
                    continue;
                }
                cache.copy(pointB1);
                if (cache.sub(pointA2).lengthSquared() < epsilon) {
                    float projected1 = directedNormal.dot(pointA1);
                    float projected2 = directedNormal.dot(pointB2);
                    if (projected1>projected2) {
                        pairResults.remove(j);
                    } else {
                        pairResults.remove(i);
                        i--;
                        break;
                    }
                    j--;
                    continue;
                }
            }
        }
    }
}
