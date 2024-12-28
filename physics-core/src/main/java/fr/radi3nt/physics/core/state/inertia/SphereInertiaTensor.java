package fr.radi3nt.physics.core.state.inertia;

import fr.radi3nt.maths.components.advanced.matrix.ArrayMatrix3x3;
import fr.radi3nt.maths.components.advanced.matrix.Matrix3x3;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;

public class SphereInertiaTensor implements InertiaTensor {

    private final float radius;
    private final float mass;

    public SphereInertiaTensor(float radius, float mass) {
        this.radius = radius;
        this.mass = mass;
    }

    @Override
    public Matrix3x3 getInverseTensor() {
        float factor = 3/(2*radius*radius*mass);
        Matrix3x3 tensor = ArrayMatrix3x3.newIdentity();
        tensor.scale(new SimpleVector3f(factor, factor, factor));
        return tensor;
    }
}
