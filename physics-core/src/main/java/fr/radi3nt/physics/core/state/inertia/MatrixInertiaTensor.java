package fr.radi3nt.physics.core.state.inertia;

import fr.radi3nt.maths.components.advanced.matrix.Matrix3x3;

public class MatrixInertiaTensor implements InertiaTensor {

    private final Matrix3x3 inverseTensor;

    public MatrixInertiaTensor(Matrix3x3 inverseTensor) {
        this.inverseTensor = inverseTensor;
    }

    @Override
    public Matrix3x3 getInverseTensor() {
        return inverseTensor;
    }
}
