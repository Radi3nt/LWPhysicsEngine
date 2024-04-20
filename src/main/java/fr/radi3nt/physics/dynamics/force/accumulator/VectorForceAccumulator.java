package fr.radi3nt.physics.dynamics.force.accumulator;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;

import java.util.Collections;
import java.util.Vector;

public class VectorForceAccumulator implements ForceAccumulator {

    private final Vector<Vector3f> forceAndTorque;

    public VectorForceAccumulator() {
        this.forceAndTorque = new Vector<>();
    }

    @Override
    public void empty() {
        forceAndTorque.clear();
    }

    public void setSize(int size) {
        forceAndTorque.setSize(size*2);
        forceAndTorque.replaceAll(ignored -> new SimpleVector3f());
    }

    public void setForce(ForceResult result, int i) {
        forceAndTorque.set(i*2, result.getForce());
        forceAndTorque.set(i*2+1, result.getTorque());
    }

    public void getForce(ForceResult result, int i) {
        result.set(forceAndTorque.get(i*2), forceAndTorque.get(i*2+1));
    }

    @Override
    public void addForce(ForceResult result, int i) {
        forceAndTorque.get(i*2).add(result.getForce());
        forceAndTorque.get(i*2+1).add(result.getTorque());
    }

    @Override
    public void addToAll(ForceResult force) {
        for (int i = 0; i < forceAndTorque.size()/2; i++) {
            Vector3f summedForce = forceAndTorque.get(i*2);
            Vector3f summedTorque = forceAndTorque.get(i*2+1);
            summedForce.add(force.getForce());
            summedTorque.add(force.getTorque());
        }
    }
}
