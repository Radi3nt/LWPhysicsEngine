package fr.radi3nt.physics.dynamics.ode.integrator;

import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.force.accumulator.MotionAccumulator;
import fr.radi3nt.physics.dynamics.force.accumulator.MotionResult;
import fr.radi3nt.physics.dynamics.island.EditableRigidBodyIsland;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;
import fr.radi3nt.physics.dynamics.transformer.VelocityTransformer;

public class SemiImplicitEulerIntegrator implements Integrator {

    private final VelocityTransformer transformer;

    public SemiImplicitEulerIntegrator(VelocityTransformer transformer) {
        this.transformer = transformer;
    }

    @Override
    public void integrate(RigidBodyIsland original, EditableRigidBodyIsland resultIsland, MotionAccumulator forces, float dt) {
        MotionResult result = new MotionResult();

        for (int i = 0; i < original.getSize(); i++) {
            RigidBody currentBody = original.getRigidBody(i);

            if (currentBody.isStatic()) {
                resultIsland.setRigidBody(currentBody.preview(), i);
                continue;
            }

            forces.getMotion(result, i);

            Vector3f newLinearMomentum = Integration.integrateVector(result.getLinear().duplicate(), currentBody.getDynamicsData().getLinearMomentum(), dt);
            Vector3f newAngularMomentum = Integration.integrateVector(result.getAngular().duplicate(), currentBody.getDynamicsData().getAngularMomentum(), dt);

            resultIsland.setRigidBody(currentBody.preview(newLinearMomentum, newAngularMomentum), i);
        }

        transformer.transform(resultIsland, dt);

        for (int i = 0; i < original.getSize(); i++) {
            RigidBody currentBody = resultIsland.getRigidBody(i);
            if (currentBody.isStatic()) {
                continue;
            }

            Vector3f newLinearMomentum = currentBody.getDynamicsData().getLinearMomentum();
            Vector3f newAngularMomentum = currentBody.getDynamicsData().getAngularMomentum();

            Vector3f newLinearVelocity = currentBody.getDynamicsData().toLinearVelocity(newLinearMomentum.duplicate());
            Vector3f newAngularVelocity = currentBody.getDynamicsData().toAngularVelocity(newAngularMomentum.duplicate());

            Vector3f newPosition = Integration.integrateVector(newLinearVelocity, currentBody.getDynamicsData().getPosition(), dt);
            Quaternion newRotation = Integration.integrateQuaternion(newAngularVelocity, currentBody.getDynamicsData().getRotation(), dt);

            resultIsland.setRigidBody(currentBody.preview(newPosition, newRotation, newLinearMomentum, newAngularMomentum), i);
        }
    }

}
