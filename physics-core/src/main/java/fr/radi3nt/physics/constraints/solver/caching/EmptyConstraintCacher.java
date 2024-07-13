package fr.radi3nt.physics.constraints.solver.caching;

import fr.radi3nt.maths.components.arbitrary.VectorNf;
import fr.radi3nt.physics.constraints.constraint.caching.CachingConstraintModule;

public class EmptyConstraintCacher implements ConstraintCacher {

    @Override
    public void prepare(CachingConstraintModule[] constraintModules) {

    }

    @Override
    public void cache(CachingConstraintModule[] constraintModules, VectorNf lambda) {

    }
}
