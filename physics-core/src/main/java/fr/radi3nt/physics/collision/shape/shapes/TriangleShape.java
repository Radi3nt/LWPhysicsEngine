package fr.radi3nt.physics.collision.shape.shapes;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.AABB;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.OBBDynamicAABB;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.VerticesAABB;
import fr.radi3nt.physics.collision.detection.broad.pre.PreCollisionShape;
import fr.radi3nt.physics.collision.detection.broad.sphere.BoundingSphere;
import fr.radi3nt.physics.collision.detection.broad.sphere.SetBoundingSphere;
import fr.radi3nt.physics.core.TransformedObject;

import java.util.Objects;

public class TriangleShape implements CollisionShape, PreCollisionShape {

    private final boolean oneWay, edgeless;
    private final Vector3f vertex1, vertex2, vertex3;

    private final Vector3f center;
    private final float radius;

    public TriangleShape(boolean oneWay, boolean edgeless, Vector3f vertex1, Vector3f vertex2, Vector3f vertex3) {
        this.oneWay = oneWay;
        this.edgeless = edgeless;
        this.vertex1 = vertex1;
        this.vertex2 = vertex2;
        this.vertex3 = vertex3;


        center = getCircumcenter();
        radius = center.duplicate().sub(vertex1).length();
    }

    public Vector3f getCircumcenter() {
        Vector3f v21 = vertex2.duplicate().sub(vertex1);
        Vector3f v12 = vertex1.duplicate().sub(vertex2);
        Vector3f v13 = vertex1.duplicate().sub(vertex3);
        Vector3f v23 = vertex2.duplicate().sub(vertex3);
        Vector3f v31 = vertex3.duplicate().sub(vertex1);
        Vector3f v32 = vertex3.duplicate().sub(vertex2);

        float divisor = (2*v12.duplicate().cross(v23).lengthSquared());

        float alpha = v23.lengthSquared()*v12.dot(v13)/divisor;
        float beta = v13.lengthSquared()*v21.dot(v23)/divisor;
        float gamma = v12.lengthSquared()*v31.dot(v32)/divisor;

        Vector3f center = new SimpleVector3f();
        center.add(vertex1.duplicate().mul(alpha));
        center.add(vertex2.duplicate().mul(beta));
        center.add(vertex3.duplicate().mul(gamma));

        return center;
    }

    @Override
    public AABB getBoundingBox(TransformedObject object) {
        return VerticesAABB.from(object, vertex1, vertex2, vertex3);
    }

    @Override
    public BoundingSphere getBoundingSphere(TransformedObject object) {
        Vector3f circumcenter = center.duplicate();
        object.getRotation().transform(circumcenter);
        return SetBoundingSphere.from(object, circumcenter, radius);
    }

    public boolean isOneWay() {
        return oneWay;
    }

    public boolean isEdgeless() {
        return edgeless;
    }

    public Vector3f getVertex1() {
        return vertex1;
    }

    public Vector3f getVertex2() {
        return vertex2;
    }

    public Vector3f getVertex3() {
        return vertex3;
    }

    @Override
    public final boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof TriangleShape)) return false;

        TriangleShape that = (TriangleShape) o;
        return oneWay == that.oneWay && edgeless == that.edgeless && Objects.equals(vertex1, that.vertex1) && Objects.equals(vertex2, that.vertex2) && Objects.equals(vertex3, that.vertex3);
    }

    @Override
    public int hashCode() {
        int result = Boolean.hashCode(oneWay);
        result = 31 * result + Boolean.hashCode(edgeless);
        result = 31 * result + Objects.hashCode(vertex1);
        result = 31 * result + Objects.hashCode(vertex2);
        result = 31 * result + Objects.hashCode(vertex3);
        return result;
    }
}
