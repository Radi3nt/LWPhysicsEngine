package fr.radi3nt.physics.dynamics.ode.rk4;

import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.force.accumulator.MotionResult;
import fr.radi3nt.physics.dynamics.force.caster.ForceCaster;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;
import fr.radi3nt.physics.dynamics.ode.integrator.Integration;
import fr.radi3nt.physics.dynamics.ode.integrator.Integrator;

public class AverageRungeKutta4OdeSolver extends RungeKutta4OdeSolver {

    public AverageRungeKutta4OdeSolver(Integrator integrator, ForceCaster forceCaster) {
        super(integrator, forceCaster);
    }

    @Override
    protected void combineResults(RigidBodyIsland rigidBodyIsland, float dt) {
        int size = rigidBodyIsland.getSize();
        Vector3f[] averageForcesAndTorque = new Vector3f[size*2];
        Vector3f[] averageLinearAndAngularVelocities = new Vector3f[size*2];

        MotionResult forceResult = new MotionResult();
        for (int i = 0; i < size; i++) {
            fillForces(forceResult, i, averageForcesAndTorque);
            fillVelocities(rigidBodyIsland, q1, q2, q3, i, averageLinearAndAngularVelocities);
        }

        buildResultIsland(rigidBodyIsland, averageForcesAndTorque, averageLinearAndAngularVelocities, dt);
    }

    private void fillVelocities(RigidBodyIsland rigidBodyIsland, RigidBodyIsland q1, RigidBodyIsland q2, RigidBodyIsland q3, int i, Vector3f[] averageLinearAndAngularVelocities) {
        Vector3f cumulatedLinearVelocity = new SimpleVector3f();
        Vector3f cumulatedAngularVelocity = new SimpleVector3f();

        addToVelocities(rigidBodyIsland, cumulatedLinearVelocity, cumulatedAngularVelocity, i, 1/6f);
        addToVelocities(q1, cumulatedLinearVelocity, cumulatedAngularVelocity, i, 2/6f);
        addToVelocities(q2, cumulatedLinearVelocity, cumulatedAngularVelocity, i, 2/6f);
        addToVelocities(q3, cumulatedLinearVelocity, cumulatedAngularVelocity, i, 1/6f);

        averageLinearAndAngularVelocities[i*2] = cumulatedLinearVelocity;
        averageLinearAndAngularVelocities[i*2+1] = cumulatedAngularVelocity;
    }

    private void addToVelocities(RigidBodyIsland rigidBodyIsland, Vector3f cumulatedLinearVelocity, Vector3f cumulatedAngularVelocity, int i, float weight) {
        RigidBody body = rigidBodyIsland.getRigidBody(i);
        cumulatedLinearVelocity.add(body.getDynamicsData().getLinearVelocity().duplicate().mul(weight));
        cumulatedAngularVelocity.add(body.getDynamicsData().getAngularVelocity().duplicate().mul(weight));
    }

    private void fillForces(MotionResult forceResult, int i, Vector3f[] averageForcesAndTorque) {
        Vector3f cumulatedForce = new SimpleVector3f();
        Vector3f cumulatedTorque = new SimpleVector3f();

        addToForces(forceResult, k1, cumulatedForce, cumulatedTorque, i, 1/6f);
        addToForces(forceResult, k2, cumulatedForce, cumulatedTorque, i, 2/6f);
        addToForces(forceResult, k3, cumulatedForce, cumulatedTorque, i, 2/6f);
        addToForces(forceResult, k4, cumulatedForce, cumulatedTorque, i, 1/6f);

        averageForcesAndTorque[i*2] = cumulatedForce;
        averageForcesAndTorque[i*2+1] = cumulatedTorque;
    }


    private void buildResultIsland(RigidBodyIsland original, Vector3f[] averageForcesAndTorque, Vector3f[] averageLinearAndAngularVelocities, float dt) {
        result.setSize(original.getSize());
        int sleeping = 0;
        for (int i = 0; i < original.getSize(); i++) {
            RigidBody rigidBody = original.getRigidBody(i);
            RigidBody resultBody;

            Vector3f force = averageForcesAndTorque[i * 2];
            Vector3f torque = averageForcesAndTorque[i * 2+1];
            rigidBody.getSleepingData().ode(force, torque);

            if (rigidBody.isStatic()) {
                resultBody = rigidBody.preview();
                sleeping++;
            } else {
                Vector3f linearMomentum = Integration.integrateVector(force, rigidBody.getDynamicsData().getLinearMomentum(), dt);
                Vector3f angularMomentum = Integration.integrateVector(torque, rigidBody.getDynamicsData().getAngularMomentum(), dt);

                Vector3f position = Integration.integrateVector(averageLinearAndAngularVelocities[i * 2], rigidBody.getDynamicsData().getPosition(), dt);
                Quaternion rotation = Integration.integrateQuaternion(averageLinearAndAngularVelocities[i * 2 + 1], rigidBody.getDynamicsData().getRotation(), dt);

                resultBody = rigidBody.preview(position, rotation, linearMomentum, angularMomentum);
            }
            result.setRigidBody(resultBody, i);
        }
        //System.out.println("sleeping: " + sleeping + "/" + original.getSize());
    }


}
