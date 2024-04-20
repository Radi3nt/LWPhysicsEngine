package fr.radi3nt.physics.dynamics.force.accumulator;

import fr.radi3nt.maths.components.vectors.Vector3f;

public class ForceResult {

    private Vector3f force;
    private Vector3f torque;

    public void set(Vector3f force, Vector3f torque) {
        this.force = force;
        this.torque = torque;
    }

    public Vector3f getForce() {
        return force;
    }

    public Vector3f getTorque() {
        return torque;
    }

    @Override
    public String toString() {
        return "ForceResult{" +
                "force=" + force +
                ", torque=" + torque +
                '}';
    }
}
