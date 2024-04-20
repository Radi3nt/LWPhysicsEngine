package fr.radi3nt.physics.multithread.link.group;

import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.multithread.link.group.RigidBodyGroup;
import fr.radi3nt.physics.multithread.link.tree.BodyRelationLinkTree;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;

public class BodyRelationLinkTreeGrouper implements RelationLinkTreeGrouper {

    protected final BodyRelationLinkTree bodyRelationLinkTree;

    public BodyRelationLinkTreeGrouper(BodyRelationLinkTree bodyRelationLinkTree) {
        this.bodyRelationLinkTree = bodyRelationLinkTree;
    }


    public Collection<RigidBodyGroup> find(RigidBody[] bodies) {
        List<RigidBody> remainingRigidBodies = new ArrayList<>(Arrays.asList(bodies));
        Collection<RigidBodyGroup> groups = new ArrayList<>();

        RigidBodyGroup group = new RigidBodyGroup();
        while (!remainingRigidBodies.isEmpty()) {
            RigidBody current = remainingRigidBodies.get(0);
            tryAddToGroup(remainingRigidBodies, group, current);

            groups.add(group);
            group = new RigidBodyGroup();
        }
        return groups;
    }

    private void tryAddToGroup(List<RigidBody> remainingRigidBodies, RigidBodyGroup group, RigidBody current) {
        if (remainingRigidBodies.remove(current)) {
            group.group.add(current);

            Collection<RigidBody> links = bodyRelationLinkTree.getRelations(current);
            for (RigidBody rigidBody : links) {
                tryAddToGroup(remainingRigidBodies, group, rigidBody);
            }
        }
    }
}
