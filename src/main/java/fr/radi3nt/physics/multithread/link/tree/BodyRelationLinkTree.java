package fr.radi3nt.physics.multithread.link.tree;

import fr.radi3nt.physics.core.state.RigidBody;

import java.util.Collection;

public interface BodyRelationLinkTree {

    Collection<RigidBody> getRelations(RigidBody rigidBody);

}
