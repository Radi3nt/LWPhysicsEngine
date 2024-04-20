package fr.radi3nt.physics.multithread.link.tree;

import fr.radi3nt.physics.constraints.constraint.Constraint;
import fr.radi3nt.physics.constraints.constraint.index.RigidBodyIndex;
import fr.radi3nt.physics.constraints.constraint.list.ConstraintList;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;
import fr.radi3nt.physics.multithread.link.tree.BodyRelationLinkTree;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;

public class ConstraintsBodyRelationLinkTree implements BodyRelationLinkTree {

    private final ConstraintList constraintList;
    private RigidBodyIsland mainIsland;

    public ConstraintsBodyRelationLinkTree(ConstraintList constraintList) {

        this.constraintList = constraintList;
    }

    public void setMainIsland(RigidBodyIsland mainIsland) {
        this.mainIsland = mainIsland;
    }

    @Override
    public Collection<RigidBody> getRelations(RigidBody rigidBody) {
        List<RigidBody> list = new ArrayList<>();
        for (Constraint allConstraint : constraintList.getConstraints()) {
            boolean oneConcernedBody = false;
            RigidBodyIndex[] indices = allConstraint.getConcernedBodies();
            for (RigidBodyIndex concernedBody : indices) {
                if (concernedBody.getRigidBodyId(mainIsland)==rigidBody.getRigidBodyId()) {
                    oneConcernedBody = true;
                }
            }
            if (!oneConcernedBody) continue;

            for (RigidBodyIndex concernedBody : indices) {
                if (concernedBody.getRigidBodyId(mainIsland) != rigidBody.getRigidBodyId()) {

                }
            }
        }
        return list;
    }
}
