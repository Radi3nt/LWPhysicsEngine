package fr.radi3nt.physics.multithread.link.group;

import fr.radi3nt.physics.constraints.constraint.Constraint;
import fr.radi3nt.physics.constraints.constraint.list.ConstraintList;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.multithread.link.tree.ListConstraintsBodyRelationLinkTree;

import java.util.*;

public class ConstraintBodyRelationLinkTreeGrouper implements RelationLinkTreeGrouper {

    private final ListConstraintsBodyRelationLinkTree bodyRelationLinkTree;
    private final ConstraintList constraintList;

    public ConstraintBodyRelationLinkTreeGrouper(ListConstraintsBodyRelationLinkTree bodyRelationLinkTree, ConstraintList constraintList) {
        this.bodyRelationLinkTree = bodyRelationLinkTree;
        this.constraintList = constraintList;
    }


    @Override
    public Collection<RigidBodyGroup> find(RigidBody[] bodies) {
        List<RigidBody> remainingRigidBodies = new ArrayList<>(Arrays.asList(bodies));
        remainingRigidBodies.removeIf(remainingRigidBody -> remainingRigidBody.getDynamicsData().getBodyProperties().inverseMass==0 || remainingRigidBody.getSleepingData().isSleeping());

        List<Constraint> remainingConstraints = constraintList.getConstraints();
        Collection<RigidBodyGroup> groups = new ArrayList<>();

        RigidBodyGroup group = new RigidBodyGroup();
        while (!remainingRigidBodies.isEmpty()) {
            RigidBody current = remainingRigidBodies.get(0);
            tryAddToGroup(remainingRigidBodies, remainingConstraints, group, current);

            groups.add(group);
            group = new RigidBodyGroup();
        }
        return groups;
    }

    private void tryAddToGroup(List<RigidBody> remainingRigidBodies, List<Constraint> remainingConstraints, RigidBodyGroup group, RigidBody current) {
        if (remainingRigidBodies.remove(current)) {
            group.group.add(current);

            Collection<RigidBody> links = new ArrayList<>();
            bodyRelationLinkTree.getRelations(current, remainingConstraints, links, group.constraints);
            for (RigidBody rigidBody : links) {
                tryAddToGroup(remainingRigidBodies, remainingConstraints, group, rigidBody);
            }
        }
    }
}
