package fr.radi3nt.physics.dynamics.ode;

import fr.radi3nt.physics.dynamics.force.caster.ForceCaster;
import fr.radi3nt.physics.dynamics.force.accumulator.VectorForceAccumulator;
import fr.radi3nt.physics.dynamics.island.ArrayListRigidBodyIsland;
import fr.radi3nt.physics.dynamics.island.ArrayRigidBodyIsland;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;
import fr.radi3nt.physics.dynamics.island.VectorRigidBodyIsland;
import fr.radi3nt.physics.dynamics.ode.integrator.Integrator;

public class IntegratorOdeSolver implements OdeSolver {

    private final Integrator integrator;
    private final ForceCaster forceCaster;

    private final VectorForceAccumulator forceAccumulator = new VectorForceAccumulator();
    private final ArrayRigidBodyIsland result = new ArrayRigidBodyIsland();

    public IntegratorOdeSolver(Integrator integrator, ForceCaster forceCaster) {
        this.forceCaster = forceCaster;
        this.integrator = integrator;
    }

    @Override
    public RigidBodyIsland integrate(RigidBodyIsland rigidBodyIsland, float dt) {
        forceAccumulator.setSize(rigidBodyIsland.getSize());
        result.setSize(rigidBodyIsland.getSize());

        forceCaster.cast(forceAccumulator, rigidBodyIsland, dt);
        integrator.integrate(rigidBodyIsland, result, forceAccumulator, dt);

        return result;
    }
}
