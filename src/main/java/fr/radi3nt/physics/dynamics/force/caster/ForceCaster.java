package fr.radi3nt.physics.dynamics.force.caster;

import fr.radi3nt.physics.dynamics.force.accumulator.ForceAccumulator;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

public interface ForceCaster {

    void cast(ForceAccumulator accumulator, RigidBodyIsland island, float dt);

}
