package fr.radi3nt.physics.collision.detection.narrow;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.contact.cache.PersistentManifoldCache;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;
import fr.radi3nt.physics.collision.shape.shapes.CollisionShape;

public interface NarrowPhaseDetectionAlgorithm {

    PersistentManifold buildManifolds(PersistentManifoldCache manifoldCache, GeneratedContactPair pairs, long currentStep);
    boolean isSupported(CollisionShape collisionShape);

}
