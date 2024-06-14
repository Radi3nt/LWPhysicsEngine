package fr.radi3nt.physics.constraints.sle.lambda;

import fr.radi3nt.maths.components.arbitrary.VectorNf;
import fr.radi3nt.maths.components.arbitrary.vector.ArrayVectorNf;

public class NewLambdaProvider implements LambdaProvider {

    @Override
    public VectorNf newLambda(int size) {
        return new ArrayVectorNf(size);
    }

}
