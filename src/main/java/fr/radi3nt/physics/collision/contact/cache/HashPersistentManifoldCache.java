package fr.radi3nt.physics.collision.contact.cache;

import fr.radi3nt.physics.collision.contact.ContactPair;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;

import java.util.*;

public class HashPersistentManifoldCache implements PersistentManifoldCache {

    private static final int RELEVANT_STEP_DISCARD = 60;
    private final Map<ContactPair, PersistentManifold> persistentManifoldMap = new HashMap<>();

    private long currentStep;

    @Override
    public Optional<PersistentManifold> getCachedManifold(ContactPair contactPair) {
        PersistentManifold manifold = persistentManifoldMap.get(contactPair);
        if (manifold!=null)
            manifold.setRelevant(currentStep);
        return Optional.ofNullable(manifold);
    }

    @Override
    public PersistentManifold newManifold(ContactPair contactPair) {
        PersistentManifold persistentManifold = persistentManifoldMap.computeIfAbsent(contactPair, PersistentManifold::new);
        persistentManifold.setRelevant(currentStep);
        return persistentManifold;
    }

    @Override
    public void releaseManifold(ContactPair pair) {
        persistentManifoldMap.remove(pair);
    }

    public void cleanup(long step) {
        Iterator<Map.Entry<ContactPair, PersistentManifold>> iterator = persistentManifoldMap.entrySet().iterator();
        while (iterator.hasNext()) {
            Map.Entry<ContactPair, PersistentManifold> next = iterator.next();
            if (!next.getValue().isRelevant(step, RELEVANT_STEP_DISCARD)) {
                iterator.remove();
                continue;
            }
            /*
            next.getValue().removeInvalidPoints();
            if (next.getValue().getManifoldPoints().isEmpty())
                iterator.remove();
             */
        }
    }

    public void setCurrentStep(long currentStep) {
        this.currentStep = currentStep;
    }

    public Collection<PersistentManifold> getManifolds() {
        return persistentManifoldMap.values();
    }
}
