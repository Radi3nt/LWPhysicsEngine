package fr.radi3nt.physics.collision.contact.cache;

import fr.radi3nt.physics.collision.contact.ContactKeyPair;
import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.*;

public class HashPersistentManifoldCache implements PersistentManifoldCache {

    private static final int RELEVANT_STEP_DISCARD = 60;
    private final Map<ContactKeyPair, PersistentManifold> persistentManifoldMap = new HashMap<>();

    @Override
    public PersistentManifold getCachedManifold(GeneratedContactPair<RigidBody> contactPair) {
        return persistentManifoldMap.get(GeneratedContactPair.toPair(contactPair));
    }

    @Override
    public PersistentManifold newManifold(GeneratedContactPair<RigidBody> contactPair) {
        return persistentManifoldMap.computeIfAbsent(GeneratedContactPair.toPair(contactPair), keyPair -> createManifold(contactPair));
    }

    private PersistentManifold createManifold(GeneratedContactPair<RigidBody> contactPair) {
        PersistentManifold persistentManifold = new PersistentManifold(contactPair);
        contactPair.objectA.getCollisionData().getCurrentCollisions().add(persistentManifold);
        contactPair.objectB.getCollisionData().getCurrentCollisions().add(persistentManifold);
        return persistentManifold;
    }

    @Override
    public void releaseManifold(GeneratedContactPair<RigidBody> pair) {
        PersistentManifold removed = persistentManifoldMap.remove(GeneratedContactPair.toPair(pair));
        if (removed!=null)
            removedManifold(removed);
    }

    private void removedManifold(PersistentManifold removed) {
        removed.getObjectA().getCollisionData().getCurrentCollisions().remove(removed);
        removed.getObjectB().getCollisionData().getCurrentCollisions().remove(removed);
    }

    public void cleanup(long step) {
        Iterator<Map.Entry<ContactKeyPair, PersistentManifold>> iterator = persistentManifoldMap.entrySet().iterator();
        while (iterator.hasNext()) {
            Map.Entry<ContactKeyPair, PersistentManifold> next = iterator.next();
            if (!next.getValue().isRelevant(step, RELEVANT_STEP_DISCARD)) {
                removedManifold(next.getValue());
                iterator.remove();
            }
        }
    }

    public Collection<PersistentManifold> getManifolds() {
        return persistentManifoldMap.values();
    }
}
