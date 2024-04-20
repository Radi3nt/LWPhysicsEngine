package fr.radi3nt.physics.dynamics.force.caster;

import fr.radi3nt.physics.dynamics.force.accumulator.ForceAccumulator;
import fr.radi3nt.physics.dynamics.force.accumulator.ForceResult;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;

public class CompositeForceCaster implements ForceCaster {

    private final Collection<ForceCaster> forceCasters;

    public CompositeForceCaster(ForceCaster... casters) {
        forceCasters = new ArrayList<>(Arrays.asList(casters));
    }

    @Override
    public void cast(ForceAccumulator accumulator, RigidBodyIsland island, float dt) {
        for (ForceCaster forceCaster : forceCasters) {
            forceCaster.cast(accumulator, island, dt);
        }
    }

    public CompositeForceCaster add(ForceCaster caster) {
        forceCasters.add(caster);
        return this;
    }

    public CompositeForceCaster remove(ForceResult caster) {
        forceCasters.remove(caster);
        return this;
    }

    public void clear() {
        forceCasters.clear();
    }
}
