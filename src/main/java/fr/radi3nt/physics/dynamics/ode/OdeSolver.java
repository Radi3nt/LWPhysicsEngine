package fr.radi3nt.physics.dynamics.ode;

import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

public interface OdeSolver {

    RigidBodyIsland integrate(RigidBodyIsland rigidBodyIsland, float dt);

}
