package fr.radi3nt.physics.dynamics.ode.integrator;

import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.vectors.Vector3f;

public final class Integration {

    public static Vector3f integrateVector(Vector3f toIntegrate, Vector3f initial, float dt) {
        return toIntegrate.mul(dt).add(initial);
    }

    public static Quaternion integrateQuaternion(Vector3f angularVelocity, Quaternion initial, float dt) {
        Quaternion newRotation = initial.duplicate();
        if (dt != 0) {
            Quaternion initialDuplicated = initial.duplicate();
            initialDuplicated.multiplyInv(angularVelocity);
            initialDuplicated.multiply(0.5f * dt);

            newRotation.add(initialDuplicated);
            newRotation.normalise();
        }

        return newRotation;
    }

}
