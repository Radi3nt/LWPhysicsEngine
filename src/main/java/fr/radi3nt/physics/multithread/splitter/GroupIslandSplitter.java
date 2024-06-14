package fr.radi3nt.physics.multithread.splitter;

import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.island.ListRigidBodyIsland;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;
import fr.radi3nt.physics.multithread.link.group.RelationLinkTreeGrouper;
import fr.radi3nt.physics.multithread.link.group.RigidBodyGroup;

import java.util.ArrayList;
import java.util.Collection;

public class GroupIslandSplitter implements IslandSplitter {

    private final RelationLinkTreeGrouper grouper;
    private final boolean includeEmptyConstraintGroups;

    public GroupIslandSplitter(RelationLinkTreeGrouper grouper, boolean includeEmptyConstraintGroups) {
        this.grouper = grouper;
        this.includeEmptyConstraintGroups = includeEmptyConstraintGroups;
    }

    @Override
    public ConstrainedIsland[] getIslands(RigidBodyIsland main) {
        RigidBody[] rigidBodies = new RigidBody[main.getSize()];
        for (int i = 0; i < rigidBodies.length; i++) {
            rigidBodies[i] = main.getRigidBody(i);
        }
        Collection<ConstrainedIsland> islands = new ArrayList<>();
        for (RigidBodyGroup rigidBodyGroup : grouper.find(rigidBodies)) {
            if (!rigidBodyGroup.constraints.isEmpty() || includeEmptyConstraintGroups)
                islands.add(new ConstrainedIsland(new ListRigidBodyIsland(rigidBodyGroup.group), rigidBodyGroup.constraints));
        }
        return islands.toArray(new ConstrainedIsland[0]);
    }
}
