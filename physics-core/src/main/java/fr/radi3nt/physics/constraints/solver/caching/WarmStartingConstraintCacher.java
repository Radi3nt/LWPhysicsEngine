package fr.radi3nt.physics.constraints.solver.caching;

import fr.radi3nt.maths.components.arbitrary.VectorNf;
import fr.radi3nt.physics.constraints.constraint.caching.CachingConstraintModule;

public class WarmStartingConstraintCacher implements ConstraintCacher {

    private CachingConstraintModule[] constraintModules;

    @Override
    public void setModules(CachingConstraintModule[] constraintModules) {
        this.constraintModules = constraintModules;
    }

    @Override
    public void prepare(VectorNf lambda) {
        for (int i = 0; i < constraintModules.length; i++) {
            lambda.set(i, constraintModules[i].getCachedValue());
        }
    }

    @Override
    public void cache(VectorNf lambda) {
        for (int i = 0; i < lambda.size(); i++) {
            constraintModules[i].setCacheValue(lambda.get(i));
        }
    }
}
