package fr.radi3nt.physics.dynamics.force.caster;

import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.force.accumulator.MotionAccumulator;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

public class ForceDataForceCaster implements ForceCaster {

    @Override
    public void cast(MotionAccumulator accumulator, RigidBodyIsland island, float dt, int index) {
        RigidBody rigidBody = island.getRigidBody(index);
        for (ForceCaster caster : rigidBody.getForceData().getCasters()) {
            caster.cast(accumulator, island, dt, index);
        }
    }

    @Override
    public void step(RigidBodyIsland island, float dt) {
        island.iterator().forEachRemaining(rigidBody -> {
            for (ForceCaster caster : rigidBody.getForceData().getCasters()) {
                caster.step(island, dt);
            }
        });
    }
}
