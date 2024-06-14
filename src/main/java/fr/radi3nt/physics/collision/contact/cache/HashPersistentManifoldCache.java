package fr.radi3nt.physics.collision.contact.cache;

import fr.radi3nt.physics.collision.contact.ContactKeyPair;
import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;

import java.util.*;

public class HashPersistentManifoldCache implements PersistentManifoldCache {

    private static final int RELEVANT_STEP_DISCARD = 60;
    private final Map<ContactKeyPair, PersistentManifold> persistentManifoldMap = new HashMap<>();

    private long currentStep;

    @Override
    public Optional<PersistentManifold> getCachedManifold(GeneratedContactPair contactPair) {
        PersistentManifold manifold = persistentManifoldMap.get(contactPair.toPair());
        if (manifold!=null)
            manifold.setRelevant(currentStep);
        return Optional.ofNullable(manifold);
    }

    @Override
    public PersistentManifold newManifold(GeneratedContactPair contactPair) {
        PersistentManifold persistentManifold = persistentManifoldMap.computeIfAbsent(contactPair.toPair(), keyPair -> createManifold(contactPair));
        persistentManifold.setRelevant(currentStep);
        return persistentManifold;
    }

    private PersistentManifold createManifold(GeneratedContactPair contactPair) {
        PersistentManifold persistentManifold = new PersistentManifold(contactPair.toPair());
        contactPair.objectA.getCollisionData().getCurrentCollisions().add(persistentManifold);
        contactPair.objectB.getCollisionData().getCurrentCollisions().add(persistentManifold);
        return persistentManifold;
    }

    @Override
    public void releaseManifold(GeneratedContactPair pair) {
        PersistentManifold removed = persistentManifoldMap.remove(pair.toPair());
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

    public void setCurrentStep(long currentStep) {
        this.currentStep = currentStep;
    }

    public Collection<PersistentManifold> getManifolds() {
        return persistentManifoldMap.values();
    }
}
