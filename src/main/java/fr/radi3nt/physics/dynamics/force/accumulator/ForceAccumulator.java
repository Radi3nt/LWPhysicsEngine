package fr.radi3nt.physics.dynamics.force.accumulator;

public interface ForceAccumulator {

    void empty();
    void setForce(ForceResult result, int i);
    void getForce(ForceResult result, int i);
    void addForce(ForceResult result, int i);
    void addToAll(ForceResult force);

}
