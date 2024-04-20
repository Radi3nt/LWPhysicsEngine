package fr.radi3nt.physics.constraints.constraint.caching;

public class SetCachingConstraintModule implements CachingConstraintModule {

    private float cache;

    @Override
    public float getCachedValue() {
        return cache;
    }

    @Override
    public void setCacheValue(float value) {
        cache = value;
    }
}
