package fr.radi3nt.physics.main;

import fr.radi3nt.maths.components.advanced.matrix.ArrayMatrix3x3;
import fr.radi3nt.maths.components.advanced.matrix.Matrix3x3;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.CollisionData;
import fr.radi3nt.physics.collision.shape.shapes.BoxShape;
import fr.radi3nt.physics.core.state.DynamicsData;
import fr.radi3nt.physics.core.state.DynamicsProperties;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.sleeping.NoSleepingData;

public class MainSimulationTesting {

    private static final float DT = 1/60f;
    private static final float SIMULATION_TIME = 4.5f;

    public static void main(String[] args) {
        Simulation simulation = new Simulation();

        Matrix3x3 iBIT = ArrayMatrix3x3.newIdentity();
        iBIT.scale(new SimpleVector3f(1/(12f*1), 1/(12f*1), 1/(12f*1)));
        DynamicsProperties dynamicsProperties = new DynamicsProperties(1, iBIT, 1, 1, 1);

        BoxShape boxShape = new BoxShape(new SimpleVector3f(1f, 1f, 1f));
        simulation.getRigidBodyIsland().add(new RigidBody(0, DynamicsData.zero(dynamicsProperties), new CollisionData(boxShape, null), new NoSleepingData()));
        //simulation.getRigidBodyIsland().add(new RigidBody(DynamicsData.from(dynamicsProperties, new SimpleVector3f(0.5f, 0.5f, 0.5f), ComponentsQuaternion.fromAxisAndAngle(new SimpleVector3f(0, 1, 0), JavaMathAngle.fromDegree(20f))), new CollisionData(collisionShape)));
        //simulation.getRigidBodyIsland().add(new RigidBody(1, DynamicsData.from(dynamicsProperties, new SimpleVector3f(0.5f, 0.5f, 0.5f), ComponentsQuaternion.zero(), new SimpleVector3f(), new SimpleVector3f(100, 0, 0)), new CollisionData(collisionShape)));

        float t = 0;
        for (; t < SIMULATION_TIME; t+=DT) {
            simulation.step(DT);
        }

        System.out.println(simulation.getRigidBodyIsland().getRigidBody(0).getDynamicsData().getPosition());
        System.out.println(simulation.getRigidBodyIsland().getRigidBody(0).getDynamicsData().getLinearVelocity());

        for (; t >= 0; t-=DT) {
            simulation.step(-DT);
        }

        System.out.println(simulation.getRigidBodyIsland().getRigidBody(0).getDynamicsData().getPosition());

    }

}
