package fr.radi3nt.physics.dynamics.force.caster;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.force.accumulator.ForceAccumulator;
import fr.radi3nt.physics.dynamics.force.accumulator.ForceResult;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

public class FluidDragForceCaster implements ForceCaster {

    private final DragProperties dragProperties;
    private final DragCoefficientSupplier dragCoefficientSupplier;
    private final ForceResult cachingForceResult = new ForceResult();

    public FluidDragForceCaster(DragProperties dragProperties, DragCoefficientSupplier dragCoefficientSupplier) {
        this.dragProperties = dragProperties;
        this.dragCoefficientSupplier = dragCoefficientSupplier;
    }

    @Override
    public void cast(ForceAccumulator accumulator, RigidBodyIsland island, float dt) {
        for (int i = 0; i < island.getSize(); i++) {
            RigidBody rigidBody = island.getRigidBody(i);
            Vector3f dragForce = getDragForce(rigidBody);

            cachingForceResult.set(dragForce, rigidBody.getDynamicsData().getAngularMomentum().duplicate().mul(-0.5f));
            accumulator.addForce(cachingForceResult, i);
        }
    }

    private Vector3f getDragForce(RigidBody rigidBody) {
        Vector3f relativeVel = dragProperties.velocityOfFluid.duplicate().sub(rigidBody.getDynamicsData().getLinearVelocity());
        Vector3f dragForce = relativeVel.duplicate().mul(relativeVel);
        dragForce.mul(Math.signum(relativeVel.getX()), Math.signum(relativeVel.getY()), Math.signum(relativeVel.getZ()));
        dragForce.mul(1/2f*dragProperties.densityOfFluid*dragCoefficientSupplier.getDragCoefficient(rigidBody)* dragCoefficientSupplier.getSurfaceCoefficient(rigidBody));
        return dragForce;
    }

    public static class DragProperties {

        public float densityOfFluid;
        public Vector3f velocityOfFluid;

        public DragProperties(float densityOfFluid, Vector3f velocityOfFluid) {
            this.densityOfFluid = densityOfFluid;
            this.velocityOfFluid = velocityOfFluid;
        }
    }

    public interface DragCoefficientSupplier {

        float getSurfaceCoefficient(RigidBody body);
        float getDragCoefficient(RigidBody body);
    }
}
