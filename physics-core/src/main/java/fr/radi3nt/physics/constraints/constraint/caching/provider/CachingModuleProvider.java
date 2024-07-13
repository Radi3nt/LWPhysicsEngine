package fr.radi3nt.physics.constraints.constraint.caching.provider;

import fr.radi3nt.physics.constraints.constraint.caching.CachingConstraintModule;

public interface CachingModuleProvider {

    CachingConstraintModule get(int slot);

}
