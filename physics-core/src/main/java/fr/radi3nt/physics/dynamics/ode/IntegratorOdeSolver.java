package fr.radi3nt.physics.dynamics.ode;

import fr.radi3nt.physics.dynamics.force.accumulator.MotionAccumulator;
import fr.radi3nt.physics.dynamics.force.accumulator.VectorMotionAccumulator;
import fr.radi3nt.physics.dynamics.force.caster.ForceCaster;
import fr.radi3nt.physics.dynamics.island.ArrayRigidBodyIsland;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;
import fr.radi3nt.physics.dynamics.ode.integrator.Integrator;

public class IntegratorOdeSolver implements OdeSolver {

    private final Integrator integrator;
    private final ForceCaster forceCaster;

    private final VectorMotionAccumulator forceAccumulator = new VectorMotionAccumulator();
    private final ArrayRigidBodyIsland result = new ArrayRigidBodyIsland();

    public IntegratorOdeSolver(Integrator integrator, ForceCaster forceCaster) {
        this.forceCaster = forceCaster;
        this.integrator = integrator;
    }

    @Override
    public RigidBodyIsland integrate(RigidBodyIsland rigidBodyIsland, float dt) {
        forceAccumulator.setSize(rigidBodyIsland.getSize());
        result.setSize(rigidBodyIsland.getSize());

        cast(forceCaster, forceAccumulator, rigidBodyIsland, dt);
        integrator.integrate(rigidBodyIsland, result, forceAccumulator, dt);

        forceCaster.step(rigidBodyIsland, dt);

        return result;
    }

    public static void cast(ForceCaster forceCaster, MotionAccumulator accumulator, RigidBodyIsland rigidBodyIsland, float dt) {
        for (int i = 0; i < rigidBodyIsland.getSize(); i++) {
            forceCaster.cast(accumulator, rigidBodyIsland, dt, i);
        }
    }
}
