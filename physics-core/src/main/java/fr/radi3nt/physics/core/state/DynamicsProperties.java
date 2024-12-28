package fr.radi3nt.physics.core.state;

import fr.radi3nt.maths.components.advanced.matrix.Matrix3x3;

import java.util.Objects;

public class DynamicsProperties {

    public float inverseMass;
    public Matrix3x3 bodyInverseInertiaTensor;
    public float bouncingFactor;
    public float kineticFrictionFactor;
    public float staticFrictionFactor;
    public float staticFrictionThreshold;

    public DynamicsProperties(float inverseMass, Matrix3x3 bodyInverseInertiaTensor, float bouncingFactor, float kineticFrictionFactor, float staticFrictionFactor, float staticFrictionThreshold) {
        this.inverseMass = inverseMass;
        this.bodyInverseInertiaTensor = bodyInverseInertiaTensor;
        this.bouncingFactor = bouncingFactor;
        this.kineticFrictionFactor = kineticFrictionFactor;
        this.staticFrictionFactor = staticFrictionFactor;
        this.staticFrictionThreshold = staticFrictionThreshold;
    }

    public void copy(DynamicsProperties properties) {
        this.inverseMass = properties.inverseMass;
        this.bodyInverseInertiaTensor = properties.bodyInverseInertiaTensor;
        this.bouncingFactor = properties.bouncingFactor;
        this.kineticFrictionFactor = properties.kineticFrictionFactor;
        this.staticFrictionFactor = properties.staticFrictionFactor;
        this.staticFrictionThreshold = properties.staticFrictionThreshold;
    }

    @Override
    public final boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof DynamicsProperties)) return false;

        DynamicsProperties that = (DynamicsProperties) o;
        return Float.compare(inverseMass, that.inverseMass) == 0 && Float.compare(bouncingFactor, that.bouncingFactor) == 0 && Float.compare(kineticFrictionFactor, that.kineticFrictionFactor) == 0 && Float.compare(staticFrictionFactor, that.staticFrictionFactor) == 0 && Float.compare(staticFrictionThreshold, that.staticFrictionThreshold) == 0 && Objects.equals(bodyInverseInertiaTensor, that.bodyInverseInertiaTensor);
    }

    @Override
    public int hashCode() {
        int result = Float.hashCode(inverseMass);
        result = 31 * result + Objects.hashCode(bodyInverseInertiaTensor);
        result = 31 * result + Float.hashCode(bouncingFactor);
        result = 31 * result + Float.hashCode(kineticFrictionFactor);
        result = 31 * result + Float.hashCode(staticFrictionFactor);
        result = 31 * result + Float.hashCode(staticFrictionThreshold);
        return result;
    }

    @Override
    public String toString() {
        return "DynamicsProperties{" +
                "inverseMass=" + inverseMass +
                ", bodyInverseInertiaTensor=" + bodyInverseInertiaTensor +
                ", bouncingFactor=" + bouncingFactor +
                ", kineticFrictionFactor=" + kineticFrictionFactor +
                ", staticFrictionFactor=" + staticFrictionFactor +
                ", staticFrictionThreshold=" + staticFrictionThreshold +
                '}';
    }
}
