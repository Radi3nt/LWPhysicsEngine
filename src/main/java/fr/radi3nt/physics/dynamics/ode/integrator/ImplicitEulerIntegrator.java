package fr.radi3nt.physics.dynamics.ode.integrator;

import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.core.state.DynamicsData;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.force.accumulator.ForceAccumulator;
import fr.radi3nt.physics.dynamics.force.accumulator.ForceResult;
import fr.radi3nt.physics.dynamics.island.EditableRigidBodyIsland;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

public class ImplicitEulerIntegrator implements Integrator {

    @Override
    public void integrate(RigidBodyIsland original, EditableRigidBodyIsland resultIsland, ForceAccumulator forces, float dt) {
        int size = original.getSize();

        ForceResult result = new ForceResult();
        for (int i = 0; i < size; i++) {
            RigidBody currentBody = original.getRigidBody(i);

            Vector3f newPosition = Integration.integrateVector(currentBody.getDynamicsData().getLinearVelocity().duplicate(), currentBody.getDynamicsData().getPosition(), dt);
            Quaternion newRotation = Integration.integrateQuaternion(currentBody.getDynamicsData().getAngularVelocity().duplicate(), currentBody.getDynamicsData().getRotation(), dt);

            forces.getForce(result, i);
            Vector3f newLinearMomentum = Integration.integrateVector(result.getForce().duplicate(), currentBody.getDynamicsData().getLinearMomentum(), dt);
            Vector3f newAngularMomentum = Integration.integrateVector(result.getTorque().duplicate(), currentBody.getDynamicsData().getAngularMomentum(), dt);

            resultIsland.setRigidBody(new RigidBody(currentBody.getRigidBodyId(), DynamicsData.from(currentBody.getDynamicsData().getBodyProperties(), newPosition, newRotation, newLinearMomentum, newAngularMomentum), currentBody.getCollisionData(), currentBody.getSleepingData()), i);
        }
    }

}
