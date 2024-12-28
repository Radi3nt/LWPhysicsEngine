package fr.radi3nt.physics.collision.detection.narrow;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.contact.cache.PersistentManifoldCache;
import fr.radi3nt.physics.collision.contact.manifold.ManifoldPoint;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;
import fr.radi3nt.physics.collision.shape.shapes.CollisionShape;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.List;

public interface NarrowPhaseDetectionAlgorithm {

    PersistentManifold buildManifolds(PersistentManifoldCache manifoldCache, GeneratedContactPair<RigidBody> pairs, long currentStep);
    List<ManifoldPoint> buildManifoldPoints(GeneratedContactPair<?> pairs);
    boolean isSupported(CollisionShape collisionShape);

}
