package fr.radi3nt.physics.core.state.inertia;

import fr.radi3nt.maths.components.advanced.matrix.ArrayMatrix3x3;
import fr.radi3nt.maths.components.advanced.matrix.Matrix3x3;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;

public class CubeInertiaTensor implements InertiaTensor {

    private final float mass;
    private final float surface;

    public CubeInertiaTensor(float mass, float surface) {
        this.mass = mass;
        this.surface = surface;
    }

    @Override
    public Matrix3x3 getInverseTensor() {
        float factor = 6/(mass*surface*surface);
        Matrix3x3 tensor = ArrayMatrix3x3.newIdentity();
        tensor.scale(new SimpleVector3f(factor, factor, factor));
        return tensor;
    }
}
