package fr.radi3nt.physics.dynamics.force.caster;

import fr.radi3nt.physics.dynamics.force.accumulator.MotionAccumulator;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

public interface ForceCaster {

    void cast(MotionAccumulator accumulator, RigidBodyIsland island, float dt, int index);

    default void step(RigidBodyIsland island, float dt) {

    }

}
