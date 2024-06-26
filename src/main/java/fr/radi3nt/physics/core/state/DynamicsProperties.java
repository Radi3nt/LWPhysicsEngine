package fr.radi3nt.physics.core.state;

import fr.radi3nt.maths.components.advanced.matrix.Matrix3x3;

public class DynamicsProperties {

    public final float inverseMass;
    public final Matrix3x3 bodyInverseInertiaTensor;
    public float bouncingFactor;
    public float kineticFrictionFactor;
    public float staticFrictionFactor;

    public DynamicsProperties(float inverseMass, Matrix3x3 bodyInverseInertiaTensor, float bouncingFactor, float kineticFrictionFactor, float staticFrictionFactor) {
        this.inverseMass = inverseMass;
        this.bodyInverseInertiaTensor = bodyInverseInertiaTensor;
        this.bouncingFactor = bouncingFactor;
        this.kineticFrictionFactor = kineticFrictionFactor;
        this.staticFrictionFactor = staticFrictionFactor;
    }
}
