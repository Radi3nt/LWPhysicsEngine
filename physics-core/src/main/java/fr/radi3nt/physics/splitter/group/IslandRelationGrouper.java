package fr.radi3nt.physics.splitter.group;

import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

import java.util.Collection;

public interface IslandRelationGrouper {

    Collection<RigidBodyGroup> find(RigidBodyIsland main);

}
