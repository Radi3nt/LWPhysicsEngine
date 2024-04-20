package fr.radi3nt.physics.multithread.link.tree;

import fr.radi3nt.physics.core.state.RigidBody;

import java.util.*;

public class CompositionBodyRelationLinkTree implements BodyRelationLinkTree {

    private final Collection<BodyRelationLinkTree> composedOf = new ArrayList<>();

    @Override
    public Collection<RigidBody> getRelations(RigidBody rigidBody) {
        Set<RigidBody> linked = new HashSet<>();
        for (BodyRelationLinkTree bodyRelationLinkTree : composedOf) {
            linked.addAll(bodyRelationLinkTree.getRelations(rigidBody));
        }
        return linked;
    }
}
