package fr.radi3nt.physics.constraints.constraint;

import fr.radi3nt.physics.constraints.constraint.caching.provider.CachingModuleProvider;
import fr.radi3nt.physics.constraints.constraint.index.RigidBodyIndex;

public interface ConstraintData {

    float[] getMax();
    float[] getMin();

    float[] getCorrections();
    StateConstraint[] getImpulses();
    StateConstraint[] getForces();

    DriftParameters[] getDriftParameters();
    CachingModuleProvider getCachingConstraintModules();

}
