package fr.radi3nt.physics.collision.detection.narrow.dispacher;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.contact.cache.PersistentManifoldCache;
import fr.radi3nt.physics.collision.contact.manifold.ManifoldPoint;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.List;

public interface CollisionDispatcher {

    PersistentManifold dispatch(PersistentManifoldCache manifoldCache, GeneratedContactPair<RigidBody> pair, long currentStep);
    List<ManifoldPoint> dispatch(GeneratedContactPair<?> pair);

}
