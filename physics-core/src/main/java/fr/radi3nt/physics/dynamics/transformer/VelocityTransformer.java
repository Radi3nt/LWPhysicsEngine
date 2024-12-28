package fr.radi3nt.physics.dynamics.transformer;

import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

public interface VelocityTransformer {

    void transform(RigidBodyIsland result, float dt);

}
