package fr.radi3nt.physics.constraints.constraint.caching;

public class EmptyCachingConstraintModule implements CachingConstraintModule {

    public static final EmptyCachingConstraintModule INSTANCE = new EmptyCachingConstraintModule();

    private EmptyCachingConstraintModule() {
    }


    @Override
    public float getCachedValue() {
        return 0;
    }

    @Override
    public void setCacheValue(float value) {

    }
}
