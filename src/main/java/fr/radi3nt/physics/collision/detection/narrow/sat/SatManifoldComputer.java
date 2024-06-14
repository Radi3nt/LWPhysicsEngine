package fr.radi3nt.physics.collision.detection.narrow.sat;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
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

    private final Vector3f cache = new SimpleVector3f();

    public Optional<PersistentManifold> compute(PersistentManifoldCache manifoldCache, GeneratedContactPair pair, CollisionResult collisionResult) {
        Optional<PersistentManifold> result = manifoldCache.getCachedManifold(pair);

        List<ManifoldPoint> pairResults = createContactPoints(pair, collisionResult);

        if (!result.isPresent()) {
            if (!pairResults.isEmpty()) {
                PersistentManifold current = manifoldCache.newManifold(pair);
                current.refresh(pair.objectA, pair.objectB);
                current.addManifoldPoints(pairResults);
                return Optional.of(current);
            }
        } else {
            PersistentManifold manifold = result.get();

            Collection<ManifoldPoint> toRemove = new ArrayList<>();
            for (ManifoldPoint manifoldPoint : result.get().getManifoldPoints()) {
                for (ManifoldPoint pairResult : pairResults) {
                    if (manifoldPoint.index.equals(pairResult.index)) {
                        pairResult.cachedContactLambda = manifoldPoint.cachedContactLambda;
                        pairResult.cachedFrictionLambda = manifoldPoint.cachedFrictionLambda;
                        toRemove.add(manifoldPoint);
                        break;
                    }
                }
            }

            manifold.refresh(pair.objectA, pair.objectB);

            manifold.getManifoldPoints().removeAll(toRemove);
            manifold.addManifoldPoints(pairResults);

            if (result.get().isEmpty()) {
                manifoldCache.releaseManifold(pair);
                return Optional.empty();
            }
        }

        //todo notify objects of collisions or something? maybe not in here but yeah

        return result;
    }

    private List<ManifoldPoint> createContactPoints(GeneratedContactPair pair, CollisionResult collisionResult) {
        List<ManifoldPoint> pairResults = collisionResult.createPoints();

        removeDuplicatedContactsPoints(pairResults, pair, collisionResult.getWorldSpaceNormal());
        return pairResults;
    }

    private void removeDuplicatedContactsPoints(List<ManifoldPoint> pairResults, GeneratedContactPair pair, Vector3f directedNormal) {
        for (ManifoldPoint pairResult : pairResults) {
            pairResult.refresh(pair.objectA, pair.objectB);
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
