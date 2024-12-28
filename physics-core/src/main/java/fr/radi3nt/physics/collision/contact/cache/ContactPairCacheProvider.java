package fr.radi3nt.physics.collision.contact.cache;

import fr.radi3nt.physics.core.state.RigidBody;

public interface ContactPairCacheProvider {

    ContactPairCache<RigidBody> newFilledCache();

}
