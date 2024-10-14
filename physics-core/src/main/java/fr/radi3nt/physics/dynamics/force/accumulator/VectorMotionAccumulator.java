package fr.radi3nt.physics.dynamics.force.accumulator;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;

import java.util.Vector;

public class VectorMotionAccumulator implements MotionAccumulator {

    private final Vector<Vector3f> forceAndTorque;

    public VectorMotionAccumulator() {
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

    @Override
    public void setMotion(MotionResult result, int i) {
        forceAndTorque.set(i*2, result.getLinear());
        forceAndTorque.set(i*2+1, result.getAngular());
    }

    @Override
    public void getMotion(MotionResult result, int i) {
        result.set(forceAndTorque.get(i*2), forceAndTorque.get(i*2+1));
    }

    @Override
    public void addMotion(MotionResult result, int i) {
        forceAndTorque.get(i*2).add(result.getLinear());
        forceAndTorque.get(i*2+1).add(result.getAngular());
    }

    @Override
    public void addToAll(MotionResult force) {
        for (int i = 0; i < forceAndTorque.size()/2; i++) {
            Vector3f summedForce = forceAndTorque.get(i*2);
            Vector3f summedTorque = forceAndTorque.get(i*2+1);
            summedForce.add(force.getLinear());
            summedTorque.add(force.getAngular());
        }
    }
}
