package fr.radi3nt.physics.dynamics.ode.rk4;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.dynamics.force.accumulator.ForceResult;
import fr.radi3nt.physics.dynamics.force.caster.ForceCaster;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;
import fr.radi3nt.physics.dynamics.ode.integrator.Integrator;

public class IntegrateRungeKutta4OdeSolver extends RungeKutta4OdeSolver {

    public IntegrateRungeKutta4OdeSolver(Integrator integrator, ForceCaster forceCaster) {
        super(integrator, forceCaster);
    }

    @Override
    protected void combineResults(RigidBodyIsland rigidBodyIsland, float dt) {
        ForceResult forceResult = new ForceResult();
        for (int i = 0; i < rigidBodyIsland.getSize(); i++) {
            fillKr(forceResult, i);
        }

        buildResultIsland(rigidBodyIsland, dt);
    }

    private void buildResultIsland(RigidBodyIsland original, float dt) {
        result.setSize(original.getSize());
        integrator.integrate(original, result, kR, dt);
    }

    private void fillKr(ForceResult forceResult, int i) {
        Vector3f cumulatedForce = new SimpleVector3f();
        Vector3f cumulatedTorque = new SimpleVector3f();

        addToForces(forceResult, k1, cumulatedForce, cumulatedTorque, i, 1/6f);
        addToForces(forceResult, k2, cumulatedForce, cumulatedTorque, i, 2/6f);
        addToForces(forceResult, k3, cumulatedForce, cumulatedTorque, i, 2/6f);
        addToForces(forceResult, k4, cumulatedForce, cumulatedTorque, i, 1/6f);

        forceResult.set(cumulatedForce, cumulatedTorque);

        kR.setForce(forceResult, i);
    }

}
