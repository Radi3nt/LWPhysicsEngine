package fr.radi3nt.physics.multithread.link.group;

import fr.radi3nt.physics.core.state.RigidBody;

import java.util.Collection;

public interface RelationLinkTreeGrouper {


    Collection<RigidBodyGroup> find(RigidBody[] bodies);
}
