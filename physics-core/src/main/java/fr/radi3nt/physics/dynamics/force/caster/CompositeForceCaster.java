package fr.radi3nt.physics.dynamics.force.caster;

import fr.radi3nt.physics.dynamics.force.accumulator.MotionAccumulator;
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
    public void cast(MotionAccumulator accumulator, RigidBodyIsland island, float dt, int index) {
        for (ForceCaster forceCaster : forceCasters) {
            forceCaster.cast(accumulator, island, dt, index);
        }
    }

    @Override
    public void step(RigidBodyIsland island, float dt) {
        forceCasters.forEach(forceCaster -> forceCaster.step(island, dt));
    }

    public CompositeForceCaster add(ForceCaster caster) {
        forceCasters.add(caster);
        return this;
    }

    public CompositeForceCaster remove(ForceCaster caster) {
        forceCasters.remove(caster);
        return this;
    }

    public void clear() {
        forceCasters.clear();
    }
}
