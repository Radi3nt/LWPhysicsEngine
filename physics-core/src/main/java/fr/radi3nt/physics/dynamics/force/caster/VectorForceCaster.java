package fr.radi3nt.physics.dynamics.force.caster;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.dynamics.force.accumulator.ForceAccumulator;
import fr.radi3nt.physics.dynamics.force.accumulator.ForceResult;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

public class VectorForceCaster implements ForceCaster {

    private final ForceResult cachingForceResult = new ForceResult();
    private final Vector3f force;
    private final Vector3f torque;

    public VectorForceCaster(Vector3f force, Vector3f torque) {
        this.force = force;
        this.torque = torque;
    }

    @Override
    public void cast(ForceAccumulator accumulator, RigidBodyIsland island, float dt) {
        cachingForceResult.set(force, torque);
        accumulator.addToAll(cachingForceResult);
    }
}
