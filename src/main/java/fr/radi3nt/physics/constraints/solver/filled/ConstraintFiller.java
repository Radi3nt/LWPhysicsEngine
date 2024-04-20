package fr.radi3nt.physics.constraints.solver.filled;

import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

public interface ConstraintFiller {

    FilledData fill(RigidBodyIsland island);

}
