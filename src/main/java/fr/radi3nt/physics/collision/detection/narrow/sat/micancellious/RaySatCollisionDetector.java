package fr.radi3nt.physics.collision.detection.narrow.sat.micancellious;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.shape.sat.SatProjectedObject;
import fr.radi3nt.physics.collision.shape.sat.shape.SatShapeObject;
import fr.radi3nt.physics.core.TransformedObject;

import static fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.normal.NormalSatCollisionDetector.getNormalTransformedArray;
import static fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.normal.NormalSatCollisionDetector.getTransformedArray;
import static java.lang.Math.abs;

public class RaySatCollisionDetector {

    private static final SatResult NO_COLLISION = new SatResult(false, null, 0);
    private static final float EPSILON = 1e-4f;

    private Vector3f mta = null;
    private float collisionTEnter = 0;
    private float collisionTLeave = Float.POSITIVE_INFINITY;
    private int normal;

    public SatResult testCollision(SatShapeObject shape1, TransformedObject transformedObject, Vector3f world, Vector3f ray) {
        mta = null;
        collisionTEnter = 0;
        collisionTLeave = Float.POSITIVE_INFINITY;
        normal = 1;

        Vector3f[] vertices = getTransformedArray(shape1.getVertices(), transformedObject);
        Vector3f[] axis = getNormalTransformedArray(shape1.getAxis(), transformedObject);

        processCollision(axis, vertices, world, ray);
        if (this.collisionTEnter > this.collisionTLeave || this.collisionTLeave<0)
            return NO_COLLISION;

        if (processCollisionEdges(transformedObject, shape1, vertices, world, ray))
            return NO_COLLISION;
        if (this.collisionTEnter > this.collisionTLeave || this.collisionTLeave<0)
            return NO_COLLISION;

        return new SatResult(true, mta, collisionTEnter);
    }

    private boolean processCollisionEdges(TransformedObject transformedObject, SatShapeObject shape1, Vector3f[] vertices, Vector3f world, Vector3f ray) {
        Vector3f[] shape1Edges = shape1.getEdges();

        for (Vector3f shape1Edge : shape1Edges) {
            Vector3f edge1 = shape1Edge.duplicate();
            transformedObject.getRotation().transform(edge1);

            Vector3f satAxis = ray.duplicate().cross(edge1);
            satAxis.normalize();

            if (computeAxis(satAxis, vertices, world, ray))
                return true;
        }

        return false;
    }

    public double getCollisionTEnter() {
        return collisionTEnter;
    }

    public int getNormal() {
        return normal;
    }

    public double getCollisionTLeave() {
        return collisionTLeave;
    }

    private void processCollision(Vector3f[] axes, Vector3f[] vertices, Vector3f world, Vector3f ray) {
        for (Vector3f axe : axes) {
            if (computeAxis(axe, vertices, world, ray))
                return;
        }
    }

    private boolean computeAxis(Vector3f axis, Vector3f[] vertices, Vector3f world, Vector3f ray) {
        if (axis.lengthSquared() <=EPSILON || Float.isNaN(axis.lengthSquared()))
            return false;


        float point = axis.dot(world);
        float speed = axis.dot(ray);

        SatProjectedObject p1 = SatProjectedObject.project(vertices, axis);
        SatProjectedObject p2 = new SatProjectedObject(point, point);

        if (abs(speed) < EPSILON) {
            if (SatProjectedObject.getOverlap(p1, p2)==0) {
                collisionTEnter = Float.POSITIVE_INFINITY;
                collisionTLeave = Float.NEGATIVE_INFINITY;
                return true;
            }
            if (mta == null) {
                mta = axis;
                normal = SatProjectedObject.getOverlapNormal(p1, p2);
                //normal = p1.max > point ? -1 : 1;
            }
        }

        float tEnter = p1.getEnterTime(point, speed);
        float tLeave = p1.getLeaveTime(point, speed);

        /*
        if (tEnter > tLeave) {
            float oldTEnter = tEnter;
            tEnter = tLeave;
            tLeave = oldTEnter;
        }
         */

        if (this.collisionTEnter <= tEnter) {
            mta = axis;
            collisionTEnter = tEnter;
            normal = p1.getNormal(point);
            //normal = p1.max > point ? -1 : 1;
        }

        this.collisionTLeave = Math.min(this.collisionTLeave, tLeave);
        return false;
    }

}
