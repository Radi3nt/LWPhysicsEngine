package fr.radi3nt.physics.multithread.splitter;

import fr.radi3nt.physics.constraints.constraint.Constraint;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

import java.util.List;

public class ConstrainedIsland {

    public final RigidBodyIsland island;
    public final List<Constraint> constraints;

    public ConstrainedIsland(RigidBodyIsland island, List<Constraint> constraints) {
        this.island = island;
        this.constraints = constraints;
    }
}
