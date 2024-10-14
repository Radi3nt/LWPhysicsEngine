package fr.radi3nt.physics.dynamics.force.accumulator;

public interface MotionAccumulator {

    void empty();
    void setMotion(MotionResult result, int i);
    void getMotion(MotionResult result, int i);
    void addMotion(MotionResult result, int i);
    void addToAll(MotionResult force);

    void setSize(int size);
}
