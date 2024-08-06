package fr.radi3nt.physics.dynamics.force.accumulator;

import fr.radi3nt.maths.components.vectors.Vector3f;

public class MotionResult {

    private Vector3f linear;
    private Vector3f angular;

    public void set(Vector3f force, Vector3f torque) {
        this.linear = force;
        this.angular = torque;
    }

    public Vector3f getLinear() {
        return linear;
    }

    public Vector3f getAngular() {
        return angular;
    }

    @Override
    public String toString() {
        return "MotionResult{" +
                "linear=" + linear +
                ", angular=" + angular +
                '}';
    }
}
