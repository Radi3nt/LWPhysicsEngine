package fr.radi3nt.physics.dynamics.force.accumulator;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;

import java.util.Arrays;

public class ArrayMotionAccumulator implements MotionAccumulator {

    private final Vector3f[] forceAndTorque;

    public ArrayMotionAccumulator(int islandSize) {
        this.forceAndTorque = new Vector3f[islandSize*2];
    }

    @Override
    public void empty() {
        Arrays.setAll(forceAndTorque, i -> new SimpleVector3f());
    }

    public void setMotion(MotionResult result, int i) {
        forceAndTorque[i*2] = result.getLinear();
        forceAndTorque[i*2+1] = result.getAngular();
    }

    public void getMotion(MotionResult result, int i) {
        result.set(forceAndTorque[i*2], forceAndTorque[i*2+1]);
    }

    @Override
    public void addMotion(MotionResult result, int i) {
        forceAndTorque[i*2].add(result.getLinear());
        forceAndTorque[i*2+1].add(result.getAngular());
    }

    @Override
    public void addToAll(MotionResult force) {
        for (int i = 0; i < forceAndTorque.length/2; i++) {
            Vector3f summedForce = forceAndTorque[i*2];
            Vector3f summedTorque = forceAndTorque[i*2+1];
            summedForce.add(force.getLinear());
            summedTorque.add(force.getAngular());
        }
    }
}
