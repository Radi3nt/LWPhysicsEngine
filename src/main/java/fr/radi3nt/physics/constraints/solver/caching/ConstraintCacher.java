package fr.radi3nt.physics.constraints.solver.caching;

import fr.radi3nt.maths.components.arbitrary.VectorNf;
import fr.radi3nt.physics.constraints.constraint.caching.CachingConstraintModule;

public interface ConstraintCacher {

    void prepare(CachingConstraintModule[] constraintModules);
    void cache(CachingConstraintModule[] constraintModules, VectorNf lambda);

}
