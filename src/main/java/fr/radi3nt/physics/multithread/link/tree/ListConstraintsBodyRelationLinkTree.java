package fr.radi3nt.physics.multithread.link.tree;

import fr.radi3nt.physics.constraints.constraint.Constraint;
import fr.radi3nt.physics.constraints.constraint.index.RigidBodyIndex;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

import java.util.Collection;
import java.util.Iterator;
import java.util.List;

public class ListConstraintsBodyRelationLinkTree {

    private RigidBodyIsland mainIsland;

    public ListConstraintsBodyRelationLinkTree() {

    }

    public void setMainIsland(RigidBodyIsland mainIsland) {
        this.mainIsland = mainIsland;
    }

    public void getRelations(RigidBody rigidBody, List<Constraint> remainingConstraints, Collection<RigidBody> resultBodies, Collection<Constraint> resultConstraints) {
        for (Iterator<Constraint> iterator = remainingConstraints.listIterator(); iterator.hasNext(); ) {
            Constraint allConstraint = iterator.next();
            int currentBodyLocalIndex = -1;
            RigidBodyIndex[] indices = allConstraint.getConcernedBodies();
            for (int i = 0, indicesLength = indices.length; i < indicesLength; i++) {
                RigidBodyIndex concernedBody = indices[i];
                if (concernedBody.getRigidBodyId(mainIsland) == rigidBody.getRigidBodyId()) {
                    currentBodyLocalIndex = i;
                    break;
                }
            }
            if (currentBodyLocalIndex == -1) continue;

            iterator.remove();
            resultConstraints.add(allConstraint);

            for (int i = 0, indicesLength = indices.length; i < indicesLength; i++) {
                RigidBodyIndex concernedIndex = indices[i];
                RigidBody concernedBody = concernedIndex.getRigidBody(mainIsland);
                if (currentBodyLocalIndex != i) {
                    if (concernedBody.getDynamicsData().getBodyProperties().inverseMass==0)
                        continue;
                    resultBodies.add(concernedBody);
                }
            }
        }
    }
}
