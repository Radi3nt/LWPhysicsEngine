package fr.radi3nt.physics.constraints.solver;

import fr.radi3nt.physics.constraints.solver.filled.ConstraintFiller;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

public interface ConstraintSolver {

    void solve(ConstraintFiller constraintFiller, RigidBodyIsland island, float dt);

}
