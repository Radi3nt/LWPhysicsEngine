package fr.radi3nt.physics.core.state;

import fr.radi3nt.physics.dynamics.force.caster.ForceCaster;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;

public class ForceData {

    private final Collection<ForceCaster> casters = new ArrayList<>();

    public ForceData(ForceCaster... forceCasters) {
        casters.addAll(Arrays.asList(forceCasters));
    }

    public Collection<ForceCaster> getCasters() {
        return casters;
    }
}
