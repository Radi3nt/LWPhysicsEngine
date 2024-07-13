package fr.radi3nt.physics.dynamics.force.accumulator;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;

import java.util.Arrays;

public class ArrayForceAccumulator implements ForceAccumulator {

    private final Vector3f[] forceAndTorque;

    public ArrayForceAccumulator(int islandSize) {
        this.forceAndTorque = new Vector3f[islandSize*2];
    }

    @Override
    public void empty() {
        Arrays.setAll(forceAndTorque, i -> new SimpleVector3f());
    }

    public void setForce(ForceResult result, int i) {
        forceAndTorque[i*2] = result.getForce();
        forceAndTorque[i*2+1] = result.getTorque();
    }

    public void getForce(ForceResult result, int i) {
        result.set(forceAndTorque[i*2], forceAndTorque[i*2+1]);
    }

    @Override
    public void addForce(ForceResult result, int i) {
        forceAndTorque[i*2].add(result.getForce());
        forceAndTorque[i*2+1].add(result.getTorque());
    }

    @Override
    public void addToAll(ForceResult force) {
        for (int i = 0; i < forceAndTorque.length/2; i++) {
            Vector3f summedForce = forceAndTorque[i*2];
            Vector3f summedTorque = forceAndTorque[i*2+1];
            summedForce.add(force.getForce());
            summedTorque.add(force.getTorque());
        }
    }
}
