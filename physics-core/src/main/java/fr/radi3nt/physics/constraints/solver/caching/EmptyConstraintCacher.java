package fr.radi3nt.physics.constraints.solver.caching;

import fr.radi3nt.maths.components.arbitrary.VectorNf;
import fr.radi3nt.physics.constraints.constraint.caching.CachingConstraintModule;

public class EmptyConstraintCacher implements ConstraintCacher {


    @Override
    public void setModules(CachingConstraintModule[] constraintModules) {

    }

    @Override
    public void prepare(VectorNf lambda) {

    }

    @Override
    public void cache(VectorNf lambda) {

    }
}
