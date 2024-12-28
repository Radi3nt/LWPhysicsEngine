package fr.radi3nt.physics.constraints.sle.lambda;

import fr.radi3nt.maths.components.arbitrary.VectorNf;
import fr.radi3nt.maths.components.arbitrary.vector.ArrayVectorNf;
import fr.radi3nt.physics.constraints.solver.caching.WarmStartingConstraintCacher;

public class SetWarmStartingLambdaProvider implements LambdaProvider {

    private float warmStartingAmount;
    private final WarmStartingConstraintCacher cacher = new WarmStartingConstraintCacher();

    public SetWarmStartingLambdaProvider(float warmStartingAmount) {
        this.warmStartingAmount = warmStartingAmount;
    }

    public void setWarmStartingAmount(float warmStartingAmount) {
        this.warmStartingAmount = warmStartingAmount;
    }

    public WarmStartingConstraintCacher getCacher() {
        return cacher;
    }

    @Override
    public VectorNf newLambda(int size) {
        VectorNf lambda = new ArrayVectorNf(size);
        cacher.prepare(lambda);
        lambda.mul(warmStartingAmount);
        return lambda;
    }
}
