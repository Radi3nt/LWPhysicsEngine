package fr.radi3nt.physics.core.state;

import fr.radi3nt.maths.Maths;
import fr.radi3nt.maths.components.advanced.matrix.ArrayMatrix3x3;
import fr.radi3nt.maths.components.advanced.matrix.Matrix3x3;
import fr.radi3nt.maths.components.advanced.quaternions.ComponentsQuaternion;
import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;

public class DynamicsData {

    private final DynamicsProperties dynamicsProperties;

    private Vector3f position;
    private Quaternion rotation;

    private Vector3f linearMomentum;
    private Vector3f angularMomentum;

    private Vector3f linearVelocity;
    private Vector3f angularVelocity;

    private Matrix3x3 iInv;

    private DynamicsData(DynamicsProperties dynamicsProperties) {
        this.dynamicsProperties = dynamicsProperties;
    }

    public static DynamicsData empty(DynamicsProperties properties) {
        return new DynamicsData(properties);
    }

    public static DynamicsData zero(DynamicsProperties properties) {
        DynamicsData data = empty(properties);
        data.zeroAngularMomentum();
        data.zeroLinearMomentum();
        data.position = new SimpleVector3f();
        data.rotation = ComponentsQuaternion.zero();
        data.computeIInv();

        return data;
    }

    public static DynamicsData from(DynamicsProperties properties, Vector3f position, Quaternion rotation, Vector3f linearMomentum, Vector3f angularMomentum) {
        DynamicsData data = empty(properties);
        data.setPosition(position);
        data.setRotation(rotation);
        data.setLinearMomentum(linearMomentum);
        data.setAngularMomentum(angularMomentum);
        return data;
    }

    public static DynamicsData from(DynamicsProperties properties, Vector3f position) {
        DynamicsData data = empty(properties);
        data.zeroAngularMomentum();
        data.zeroLinearMomentum();
        data.position = position;
        data.rotation = ComponentsQuaternion.zero();
        data.iInv = ArrayMatrix3x3.newIdentity();
        return data;
    }

    public static DynamicsData from(DynamicsProperties properties, Vector3f position, Quaternion rotation) {
        DynamicsData data = empty(properties);
        data.setPosition(position);
        data.setRotation(rotation);
        data.zeroAngularMomentum();
        data.zeroLinearMomentum();
        return data;
    }

    public static DynamicsData from(DynamicsData dynamicsData) {
        DynamicsData data = new DynamicsData(dynamicsData.dynamicsProperties);
        data.position = dynamicsData.position.duplicate();
        data.rotation = dynamicsData.rotation.duplicate();
        data.linearVelocity = dynamicsData.linearVelocity.duplicate();
        data.angularVelocity = dynamicsData.angularVelocity.duplicate();
        data.linearMomentum = dynamicsData.linearMomentum.duplicate();
        data.angularMomentum = dynamicsData.angularMomentum.duplicate();
        data.iInv = dynamicsData.iInv.duplicate();
        return data;
    }

    public void setPosition(Vector3f position) {
        this.position = position;
    }

    public Vector3f getPosition() {
        return position;
    }

    public void setRotation(Quaternion rotation) {
        this.rotation = rotation;
        computeIInv();
    }

    public Quaternion getRotation() {
        return rotation;
    }

    public Matrix3x3 getIInv() {
        return iInv;
    }

    private void computeIInv() {
        Matrix3x3 inverseBodyInv = dynamicsProperties.bodyInverseInertiaTensor;

        if (this.iInv==null)
            this.iInv = new ArrayMatrix3x3();

        this.iInv.identity();
        Matrix3x3 rotationMatrix = this.iInv;
        rotationMatrix.quaternionRotation(rotation);

        Matrix3x3 transposedRotation = rotationMatrix.duplicate();
        transposedRotation.transpose();

        rotationMatrix.multiply(inverseBodyInv);
        rotationMatrix.multiply(transposedRotation);

        this.iInv = rotationMatrix;
    }

    public void setLinearMomentum(Vector3f linearMomentum) {
        this.linearMomentum = linearMomentum;
        computeLinearVelocity();
    }

    public void zeroLinearMomentum() {
        this.linearMomentum = new SimpleVector3f();
        this.linearVelocity = new SimpleVector3f();
    }

    public void addLinearImpulse(Vector3f impulse) {
        this.linearMomentum.add(impulse);
        computeLinearVelocity();
    }

    public void addLinearVelocity(Vector3f velocity) {
        Vector3f impulse = velocity.div(dynamicsProperties.inverseMass);
        if (Float.isNaN(impulse.lengthSquared()))
            return;
        this.linearMomentum.add(impulse);
        computeLinearVelocity();
    }

    public Vector3f toLinearVelocity(Vector3f linearMomentum) {
        return linearMomentum.duplicate().mul(dynamicsProperties.inverseMass);
    }

    private void computeLinearVelocity() {
        this.linearVelocity = toLinearVelocity(linearMomentum);
    }

    public void setAngularMomentum(Vector3f angularMomentum) {
        this.angularMomentum = angularMomentum;
        computeAngularVelocity();
    }

    public void zeroAngularMomentum() {
        this.angularMomentum = this.angularVelocity = new SimpleVector3f();
    }

    public void addAngularImpulse(Vector3f impulse) {
        this.angularMomentum.add(impulse);
        computeAngularVelocity();
    }

    public Vector3f toAngularVelocity(Vector3f angularMomentum) {
        Vector3f transformed = angularMomentum.duplicate();
        iInv.transform(transformed);
        return transformed;
    }

    private void computeAngularVelocity() {
        this.angularVelocity = toAngularVelocity(angularMomentum);
    }

    public Vector3f getLinearMomentum() {
        return linearMomentum;
    }

    public Vector3f getAngularMomentum() {
        return angularMomentum;
    }

    public Vector3f getLinearVelocity() {
        return linearVelocity;
    }

    public Vector3f getAngularVelocity() {
        return angularVelocity;
    }

    private static void clampVector(Vector3f vector, float max) {
        vector.set(Maths.clamp(vector.getX(), -max, max), Maths.clamp(vector.getY(), -max, max), Maths.clamp(vector.getZ(), -max, max));
    }

    public DynamicsProperties getBodyProperties() {
        return dynamicsProperties;
    }

    public void copyState(DynamicsData data) {
        this.position = data.position;
        this.rotation = data.rotation;
        this.iInv = data.iInv;

        this.linearMomentum = data.linearMomentum;
        this.angularMomentum = data.angularMomentum;

        this.linearVelocity = data.linearVelocity;
        this.angularVelocity = data.angularVelocity;
    }
}
