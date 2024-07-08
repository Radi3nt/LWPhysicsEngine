package fr.radi3nt.physics.splitter;

import fr.radi3nt.physics.dynamics.island.ListRigidBodyIsland;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;
import fr.radi3nt.physics.splitter.group.IslandRelationGrouper;
import fr.radi3nt.physics.splitter.group.RigidBodyGroup;

import java.util.ArrayList;
import java.util.Collection;

public class GroupIslandSplitter implements IslandSplitter {

    private final IslandRelationGrouper grouper;
    private final boolean includeEmptyConstraintGroups;

    public GroupIslandSplitter(IslandRelationGrouper grouper, boolean includeEmptyConstraintGroups) {
        this.grouper = grouper;
        this.includeEmptyConstraintGroups = includeEmptyConstraintGroups;
    }

    @Override
    public ConstrainedIsland[] getIslands(RigidBodyIsland main) {
        Collection<ConstrainedIsland> islands = new ArrayList<>();
        for (RigidBodyGroup rigidBodyGroup : grouper.find(main)) {
            if (!rigidBodyGroup.constraints.isEmpty() || includeEmptyConstraintGroups)
                islands.add(new ConstrainedIsland(new ListRigidBodyIsland(rigidBodyGroup.group), rigidBodyGroup.constraints));
        }
        return islands.toArray(new ConstrainedIsland[0]);
    }
}
