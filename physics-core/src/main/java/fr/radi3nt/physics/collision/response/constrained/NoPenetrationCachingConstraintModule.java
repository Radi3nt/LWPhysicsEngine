package fr.radi3nt.physics.collision.response.constrained;

import fr.radi3nt.physics.collision.contact.manifold.ManifoldPointData;
import fr.radi3nt.physics.constraints.constraint.caching.CachingConstraintModule;

public class NoPenetrationCachingConstraintModule implements CachingConstraintModule {

    private final ManifoldPointData data;

    public NoPenetrationCachingConstraintModule(ManifoldPointData data) {
        this.data = data;
    }

    @Override
    public float getCachedValue() {
        return data.cachedContactLambda;
    }

    @Override
    public void setCacheValue(float value) {
        data.cachedContactLambda = value;
    }
}
