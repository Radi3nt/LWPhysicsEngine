package fr.radi3nt.physics.dynamics.ode.rk4;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.dynamics.force.accumulator.MotionResult;
import fr.radi3nt.physics.dynamics.force.accumulator.VectorMotionAccumulator;
import fr.radi3nt.physics.dynamics.force.caster.ForceCaster;
import fr.radi3nt.physics.dynamics.island.ArrayRigidBodyIsland;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;
import fr.radi3nt.physics.dynamics.ode.IntegratorOdeSolver;
import fr.radi3nt.physics.dynamics.ode.OdeSolver;
import fr.radi3nt.physics.dynamics.ode.integrator.Integrator;

public abstract class RungeKutta4OdeSolver implements OdeSolver {

    private final ForceCaster forceCaster;
    protected final Integrator integrator;

    protected final VectorMotionAccumulator k1 = new VectorMotionAccumulator();
    protected final VectorMotionAccumulator k2 = new VectorMotionAccumulator();
    protected final VectorMotionAccumulator k3 = new VectorMotionAccumulator();
    protected final VectorMotionAccumulator k4 = new VectorMotionAccumulator();
    protected final VectorMotionAccumulator kR = new VectorMotionAccumulator();

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

        IntegratorOdeSolver.cast(forceCaster, k1, rigidBodyIsland, 0);

        integrator.integrate(rigidBodyIsland, q1, k1, dt/2f);
        IntegratorOdeSolver.cast(forceCaster, k2, q1, dt/2);

        integrator.integrate(rigidBodyIsland, q2, k2, dt/2f);
        IntegratorOdeSolver.cast(forceCaster, k3, q2, dt/2);

        integrator.integrate(rigidBodyIsland, q3, k3, dt);
        IntegratorOdeSolver.cast(forceCaster, k4, q3, dt);

        forceCaster.step(rigidBodyIsland, dt);

        combineResults(rigidBodyIsland, dt);

        return result;
    }

    protected abstract void combineResults(RigidBodyIsland rigidBodyIsland, float dt);

    protected void addToForces(MotionResult forceResult, VectorMotionAccumulator k, Vector3f cumulatedForce, Vector3f cumulatedTorque, int i, float weight) {
        k.getMotion(forceResult, i);
        cumulatedForce.add(forceResult.getLinear().mul(weight));
        cumulatedTorque.add(forceResult.getAngular().mul(weight));
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
