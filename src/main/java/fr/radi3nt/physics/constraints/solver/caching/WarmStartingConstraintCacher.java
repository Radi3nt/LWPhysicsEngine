package fr.radi3nt.physics.constraints.solver.caching;

import fr.radi3nt.maths.components.arbitrary.VectorNf;
import fr.radi3nt.maths.components.arbitrary.vector.ArrayVectorNf;
import fr.radi3nt.physics.constraints.constraint.caching.CachingConstraintModule;
import fr.radi3nt.physics.constraints.sle.lambda.SetWarmStartingLambdaProvider;

public class WarmStartingConstraintCacher implements ConstraintCacher {

    private final SetWarmStartingLambdaProvider lambda;

    public WarmStartingConstraintCacher(SetWarmStartingLambdaProvider lambda) {
        this.lambda = lambda;
    }

    @Override
    public void prepare(CachingConstraintModule[] constraintModules) {
        VectorNf preparedLambda = new ArrayVectorNf(constraintModules.length);
        for (int i = 0; i < constraintModules.length; i++) {
            preparedLambda.set(i, constraintModules[i].getCachedValue());
        }
        lambda.setWarmStartedLambda(preparedLambda);
    }

    @Override
    public void cache(CachingConstraintModule[] constraintModules, VectorNf lambda) {
        for (int i = 0; i < lambda.size(); i++) {
            constraintModules[i].setCacheValue(lambda.get(i));
        }
    }
}
