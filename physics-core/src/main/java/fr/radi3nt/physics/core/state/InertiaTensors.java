package fr.radi3nt.physics.core.state;

import fr.radi3nt.maths.components.advanced.matrix.ArrayMatrix3x3;
import fr.radi3nt.maths.components.advanced.matrix.Matrix3x3;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;

public class InertiaTensors {

    public static Matrix3x3 cubeInverseTensor(float mass, float surface) {
        float factor = 6/(mass*surface*surface);
        Matrix3x3 tensor = ArrayMatrix3x3.newIdentity();
        tensor.scale(new SimpleVector3f(factor, factor, factor));
        return tensor;
    }

    public static Matrix3x3 sphereInverseTensor(float mass, float radius) {
        float factor = 3/(2*radius*radius*mass);
        Matrix3x3 tensor = ArrayMatrix3x3.newIdentity();
        tensor.scale(new SimpleVector3f(factor, factor, factor));
        return tensor;
    }

    public static Matrix3x3 cubeTensor(float mass, float surface) {
        float factor = (mass*surface*surface)/6;
        Matrix3x3 tensor = ArrayMatrix3x3.newIdentity();
        tensor.scale(new SimpleVector3f(factor, factor, factor));
        return tensor;
    }

    public static Matrix3x3 cubeTensor(Vector3f offset, float mass, float surface) {
        Matrix3x3 cubeTensor = cubeTensor(mass, surface);
        Matrix3x3 newTensor = ArrayMatrix3x3.newIdentity();
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                newTensor.set(i, j, cubeTensor.get(i, j) + mass*(offset.lengthSquared()*kroneckerDelta(i, j) - offset.get(i)*offset.get(j)));
            }
        }
        return newTensor;
    }

    public static Matrix3x3 cuboidInverseTensor(float mass, Vector3f size) {
        Matrix3x3 tensor = ArrayMatrix3x3.newIdentity();
        float iX = size.getY()*size.getY()*size.getZ()*size.getZ();
        float iY = size.getX()*size.getX()*size.getZ()*size.getZ();
        float iZ = size.getX()*size.getX()*size.getY()*size.getY();
        tensor.scale(new SimpleVector3f(12/(mass*iX), 12/(mass*iY), 12/(mass*iZ)));
        return tensor;
    }

    private static float kroneckerDelta(float i, float j) {
        return i==j ? 1 : 0;
    }
}
