package fr.radi3nt.physics.constraints.constraint.caching.provider;

import fr.radi3nt.physics.constraints.constraint.caching.CachingConstraintModule;
import fr.radi3nt.physics.constraints.constraint.caching.EmptyCachingConstraintModule;

public class ArrayCachingModuleProvider implements CachingModuleProvider {

    private final CachingConstraintModule[] constraintModules;

    public ArrayCachingModuleProvider(CachingConstraintModule... constraintModules) {
        this.constraintModules = constraintModules;
    }

    @Override
    public CachingConstraintModule get(int slot) {
        return constraintModules[slot];
    }
}
