package fr.radi3nt.physics.constraints.constraint;

import fr.radi3nt.physics.constraints.constraint.caching.provider.CachingModuleProvider;

public interface ConstraintData {

    float[] getMax();
    float[] getMin();

    float[] getCorrections();
    StateConstraint[] getImpulses();

    DriftParameters[] getDriftParameters();
    CachingModuleProvider getCachingConstraintModules();

}
