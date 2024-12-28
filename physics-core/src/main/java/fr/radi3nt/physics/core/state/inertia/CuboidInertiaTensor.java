package fr.radi3nt.physics.core.state.inertia;

import fr.radi3nt.maths.components.advanced.matrix.ArrayMatrix3x3;
import fr.radi3nt.maths.components.advanced.matrix.Matrix3x3;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;

public class CuboidInertiaTensor implements InertiaTensor {

    private final Vector3f size;
    private final float mass;

    public CuboidInertiaTensor(Vector3f size, float mass) {
        this.size = size;
        this.mass = mass;
    }

    @Override
    public Matrix3x3 getInverseTensor() {
        Matrix3x3 tensor = ArrayMatrix3x3.newIdentity();
        float iX = size.getY()*size.getY()*size.getZ()*size.getZ();
        float iY = size.getX()*size.getX()*size.getZ()*size.getZ();
        float iZ = size.getX()*size.getX()*size.getY()*size.getY();
        tensor.scale(new SimpleVector3f(12/(mass*iX), 12/(mass*iY), 12/(mass*iZ)));
        return tensor;
    }
}
