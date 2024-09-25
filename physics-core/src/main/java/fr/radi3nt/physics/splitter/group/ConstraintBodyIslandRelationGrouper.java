package fr.radi3nt.physics.splitter.group;

import fr.radi3nt.physics.constraints.constraint.Constraint;
import fr.radi3nt.physics.constraints.constraint.index.RigidBodyIndex;
import fr.radi3nt.physics.constraints.constraint.list.ConstraintList;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

import java.util.*;

public class ConstraintBodyIslandRelationGrouper implements IslandRelationGrouper {

    private final ConstraintList constraintList;

    public ConstraintBodyIslandRelationGrouper(ConstraintList constraintList) {
        this.constraintList = constraintList;
    }


    @Override
    public Collection<RigidBodyGroup> find(RigidBodyIsland mainIsland) {
        RigidBody[] rigidBodies = new RigidBody[mainIsland.getSize()];
        for (int i = 0; i < rigidBodies.length; i++) {
            rigidBodies[i] = mainIsland.getRigidBody(i);
        }
        List<RigidBody> remainingRigidBodies = new ArrayList<>(Arrays.asList(rigidBodies));
        remainingRigidBodies.removeIf(ConstraintBodyIslandRelationGrouper::shouldNotLinkGroup);

        List<Constraint> remainingConstraints = new ArrayList<>(constraintList.getConstraints());
        Collection<RigidBodyGroup> groups = new ArrayList<>();

        RigidBodyGroup group = new RigidBodyGroup();
        while (!remainingRigidBodies.isEmpty()) {
            RigidBody current = remainingRigidBodies.get(0);
            tryAddToGroup(remainingRigidBodies, remainingConstraints, group, current, mainIsland);

            groups.add(group);
            group = new RigidBodyGroup();
        }
        return groups;
    }

    private static boolean shouldNotLinkGroup(RigidBody remainingRigidBody) {
        return remainingRigidBody.getDynamicsData().getBodyProperties().inverseMass == 0 || remainingRigidBody.getSleepingData().isSleeping();
    }

    private void tryAddToGroup(List<RigidBody> remainingRigidBodies, List<Constraint> remainingConstraints, RigidBodyGroup group, RigidBody current, RigidBodyIsland mainIsland) {
        if (remainingRigidBodies.remove(current)) {
            group.group.add(current);

            Collection<RigidBody> links = new ArrayList<>();
            getRelations(current, remainingConstraints, links, group.constraints, mainIsland);
            for (RigidBody rigidBody : links) {
                tryAddToGroup(remainingRigidBodies, remainingConstraints, group, rigidBody, mainIsland);
            }
        }
    }

    public void getRelations(RigidBody rigidBody, List<Constraint> remainingConstraints, Collection<RigidBody> resultBodies, Collection<Constraint> resultConstraints, RigidBodyIsland mainIsland) {
        for (Iterator<Constraint> iterator = remainingConstraints.listIterator(); iterator.hasNext(); ) {
            Constraint allConstraint = iterator.next();
            int currentBodyLocalIndex = -1;
            RigidBodyIndex[] indices = allConstraint.getConcernedBodies();
            for (int i = 0, indicesLength = indices.length; i < indicesLength; i++) {
                RigidBodyIndex concernedBody = indices[i];
                if (concernedBody.getRigidBodyId() == rigidBody.getRigidBodyId()) {
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
