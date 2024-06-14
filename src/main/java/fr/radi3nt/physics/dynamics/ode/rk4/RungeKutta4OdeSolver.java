package fr.radi3nt.physics.dynamics.ode.rk4;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.dynamics.force.accumulator.ForceResult;
import fr.radi3nt.physics.dynamics.force.accumulator.VectorForceAccumulator;
import fr.radi3nt.physics.dynamics.force.caster.ForceCaster;
import fr.radi3nt.physics.dynamics.island.ArrayRigidBodyIsland;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;
import fr.radi3nt.physics.dynamics.ode.OdeSolver;
import fr.radi3nt.physics.dynamics.ode.integrator.Integrator;

public abstract class RungeKutta4OdeSolver implements OdeSolver {

    private final ForceCaster forceCaster;
    protected final Integrator integrator;

    protected final VectorForceAccumulator k1 = new VectorForceAccumulator();
    protected final VectorForceAccumulator k2 = new VectorForceAccumulator();
    protected final VectorForceAccumulator k3 = new VectorForceAccumulator();
    protected final VectorForceAccumulator k4 = new VectorForceAccumulator();
    protected final VectorForceAccumulator kR = new VectorForceAccumulator();

    protected final ArrayRigidBodyIsland q1 = new ArrayRigidBodyIsland();
    protected final ArrayRigidBodyIsland q2 = new ArrayRigidBodyIsland();
    protected final ArrayRigidBodyIsland q3 = new ArrayRigidBodyIsland();
    protected final ArrayRigidBodyIsland q4 = new ArrayRigidBodyIsland();

    protected final ArrayRigidBodyIsland result = new ArrayRigidBodyIsland();

    public RungeKutta4OdeSolver(Integrator integrator, ForceCaster forceCaster) {
        this.forceCaster = forceCaster;
        this.integrator = integrator;
    }

    @Override
    public RigidBodyIsland integrate(RigidBodyIsland rigidBodyIsland, float dt) {
        int size = rigidBodyIsland.getSize();
        resetForceAccumulator(size);

        forceCaster.cast(k1, rigidBodyIsland, 0);

        integrator.integrate(rigidBodyIsland, q1, k1, dt/2f);
        forceCaster.cast(k2, q1, dt/2);

        integrator.integrate(rigidBodyIsland, q2, k2, dt/2f);
        forceCaster.cast(k3, q2, dt/2);

        integrator.integrate(rigidBodyIsland, q3, k3, dt);
        forceCaster.cast(k4, q3, dt);

        combineResults(rigidBodyIsland, dt);

        return result;
    }

    protected abstract void combineResults(RigidBodyIsland rigidBodyIsland, float dt);

    protected void addToForces(ForceResult forceResult, VectorForceAccumulator k, Vector3f cumulatedForce, Vector3f cumulatedTorque, int i, float weight) {
        k.getForce(forceResult, i);
        cumulatedForce.add(forceResult.getForce().mul(weight));
        cumulatedTorque.add(forceResult.getTorque().mul(weight));
    }

    private void resetForceAccumulator(int size) {
        k1.setSize(size);
        k2.setSize(size);
        k3.setSize(size);
        k4.setSize(size);
        kR.setSize(size);

        q1.setSize(size);
        q2.setSize(size);
        q3.setSize(size);
        q4.setSize(size);
    }
}
