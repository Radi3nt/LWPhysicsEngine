package fr.radi3nt.physics.collision.response.constrained;

import fr.radi3nt.physics.collision.contact.manifold.ManifoldPoint;
import fr.radi3nt.physics.constraints.constraint.caching.CachingConstraintModule;

public class FrictionCachingConstraintModule implements CachingConstraintModule {

    private final ManifoldPoint manifoldPoint;
    private final int slot;

    public FrictionCachingConstraintModule(ManifoldPoint manifoldPoint, int slot) {
        this.manifoldPoint = manifoldPoint;
        this.slot = slot;
    }

    @Override
    public float getCachedValue() {
        float cachedLambda = manifoldPoint.cachedFrictionLambda[slot];
        return cachedLambda;
    }

    @Override
    public void setCacheValue(float value) {
        manifoldPoint.cachedFrictionLambda[slot] = value;
    }
}
