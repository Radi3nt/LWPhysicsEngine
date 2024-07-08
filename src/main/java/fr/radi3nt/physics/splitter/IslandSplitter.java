package fr.radi3nt.physics.splitter;

import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

public interface IslandSplitter {

    ConstrainedIsland[] getIslands(RigidBodyIsland main);

}
