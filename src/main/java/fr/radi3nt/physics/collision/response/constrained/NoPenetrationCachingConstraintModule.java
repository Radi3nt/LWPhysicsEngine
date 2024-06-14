package fr.radi3nt.physics.collision.response.constrained;

import fr.radi3nt.physics.collision.contact.manifold.ManifoldPoint;
import fr.radi3nt.physics.constraints.constraint.caching.CachingConstraintModule;

public class NoPenetrationCachingConstraintModule implements CachingConstraintModule {

    private final ManifoldPoint manifoldPoint;

    public NoPenetrationCachingConstraintModule(ManifoldPoint manifoldPoint) {
        this.manifoldPoint = manifoldPoint;
    }

    @Override
    public float getCachedValue() {
        return manifoldPoint.cachedContactLambda;
    }

    @Override
    public void setCacheValue(float value) {
        manifoldPoint.cachedContactLambda = value;
    }
}
