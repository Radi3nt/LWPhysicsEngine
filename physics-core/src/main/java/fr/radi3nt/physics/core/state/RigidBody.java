package fr.radi3nt.physics.core.state;

import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.CollisionData;
import fr.radi3nt.physics.core.TransformedObject;
import fr.radi3nt.physics.sleeping.NoSleepingData;
import fr.radi3nt.physics.sleeping.SleepingData;

public class RigidBody implements TransformedObject {

    private final int rigidBodyId;
    private final ForceData forceData;
    private final DynamicsData dynamicsData;
    private final CollisionData collisionData;
    private SleepingData sleepingData;

    public RigidBody(int rigidBodyId, DynamicsData dynamicsData, CollisionData collisionData, SleepingData sleepingData) {
        this(rigidBodyId, new ForceData(), dynamicsData, collisionData, sleepingData);
    }

    public RigidBody(int rigidBodyId, DynamicsData dynamicsData, CollisionData collisionData) {
        this(rigidBodyId, new ForceData(), dynamicsData, collisionData, NoSleepingData.INSTANCE);
    }

    public RigidBody(int rigidBodyId, ForceData forceData, DynamicsData dynamicsData, CollisionData collisionData, SleepingData sleepingData) {
        this.rigidBodyId = rigidBodyId;
        this.forceData = forceData;
        this.dynamicsData = dynamicsData;
        this.collisionData = collisionData;
        this.sleepingData = sleepingData;
    }

    public RigidBody preview(Vector3f newPosition, Quaternion newOrientation, Vector3f newLinearMomentum, Vector3f newAngularMomentum) {
        return new RigidBody(rigidBodyId, forceData, DynamicsData.from(dynamicsData.getBodyProperties(), newPosition, newOrientation, newLinearMomentum, newAngularMomentum), collisionData, sleepingData);
    }

    public RigidBody preview() {
        return new RigidBody(rigidBodyId, forceData, DynamicsData.from(dynamicsData), collisionData, sleepingData);
    }

    public void setState(DynamicsData data) {
        this.dynamicsData.copyState(data);
    }

    public DynamicsData getDynamicsData() {
        return dynamicsData;
    }

    public void setState(Vector3f position, Quaternion quaternion, Vector3f linearMomentum, Vector3f angularMomentum) {
        dynamicsData.copyState(DynamicsData.from(dynamicsData.getBodyProperties(), position, quaternion, linearMomentum, angularMomentum));
    }

    public CollisionData getCollisionData() {
        return collisionData;
    }

    public SleepingData getSleepingData() {
        return sleepingData;
    }

    public void setSleepingData(SleepingData sleepingData) {
        this.sleepingData = sleepingData;
    }

    @Override
    public Vector3f getPosition() {
        return dynamicsData.getPosition();
    }

    public boolean isStatic() {
        return sleepingData.isSleeping() || dynamicsData.getBodyProperties().inverseMass==0;
    }

    @Override
    public Quaternion getRotation() {
        return dynamicsData.getRotation();
    }

    public int getRigidBodyId() {
        return rigidBodyId;
    }

    public ForceData getForceData() {
        return forceData;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof RigidBody)) return false;

        RigidBody rigidBody = (RigidBody) o;

        return rigidBodyId == rigidBody.rigidBodyId;
    }

    @Override
    public int hashCode() {
        return rigidBodyId;
    }
}
