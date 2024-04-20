package fr.radi3nt.physics.constraints.sle.lambda;

import fr.radi3nt.maths.components.arbitrary.VectorNf;
import fr.radi3nt.maths.components.arbitrary.vector.ArrayVectorNf;

public class SetWarmStartingLambdaProvider implements LambdaProvider {

    private float warmStartingAmount;
    private VectorNf warmStartedLambda = new ArrayVectorNf(0);

    public SetWarmStartingLambdaProvider(float warmStartingAmount) {
        this.warmStartingAmount = warmStartingAmount;
    }

    public void setWarmStartingAmount(float warmStartingAmount) {
        this.warmStartingAmount = warmStartingAmount;
    }

    public void setWarmStartedLambda(VectorNf warmStartedLambda) {
        this.warmStartedLambda = warmStartedLambda;
    }

    @Override
    public VectorNf newLambda(int size) {
        if (warmStartedLambda.size()!=size)
            throw new UnsupportedOperationException("Size doesn't match with prepared lambda");
        VectorNf lambda = warmStartedLambda.duplicate();
        lambda.mul(warmStartingAmount);
        return lambda;
    }
}
