package fr.radi3nt.physics.constraints.constraint;

import fr.radi3nt.physics.constraints.constraint.caching.CachingConstraintModule;
import fr.radi3nt.physics.constraints.constraint.index.RigidBodyIndex;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

public interface Constraint {

    ConstraintData compute(RigidBodyIsland island);
    RigidBodyIndex[] getConcernedBodies();
}
