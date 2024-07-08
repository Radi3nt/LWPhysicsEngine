package fr.radi3nt.physics.collision.response.constrained;

import fr.radi3nt.physics.collision.contact.manifold.ManifoldPointData;
import fr.radi3nt.physics.constraints.constraint.caching.CachingConstraintModule;

public class FrictionCachingConstraintModule implements CachingConstraintModule {

    private final ManifoldPointData data;
    private final int slot;

    public FrictionCachingConstraintModule(ManifoldPointData data, int slot) {
        this.data = data;
        this.slot = slot;
    }

    @Override
    public float getCachedValue() {
        return data.cachedFrictionLambda[slot];
    }

    @Override
    public void setCacheValue(float value) {
        data.cachedFrictionLambda[slot] = value;
    }
}
