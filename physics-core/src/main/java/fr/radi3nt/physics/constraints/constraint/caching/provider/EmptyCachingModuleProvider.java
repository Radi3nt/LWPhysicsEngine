package fr.radi3nt.physics.constraints.constraint.caching.provider;

import fr.radi3nt.physics.constraints.constraint.caching.CachingConstraintModule;
import fr.radi3nt.physics.constraints.constraint.caching.EmptyCachingConstraintModule;

public class EmptyCachingModuleProvider implements CachingModuleProvider {

    public static final EmptyCachingModuleProvider INSTANCE = new EmptyCachingModuleProvider();

    private EmptyCachingModuleProvider() {
    }

    @Override
    public CachingConstraintModule get(int slot) {
        return EmptyCachingConstraintModule.INSTANCE;
    }
}
