package fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.normal;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.maths.pool.ListVector3fPool;
import fr.radi3nt.maths.pool.ObjectPool;
import fr.radi3nt.maths.pool.QueuedPool;
import fr.radi3nt.maths.pool.Vector3fPool;
import fr.radi3nt.physics.collision.contact.ContactPair;
import fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.CollisionDetector;
import fr.radi3nt.physics.collision.shape.sat.SatProjectedObject;
import fr.radi3nt.physics.collision.shape.sat.shape.SatShapeObject;
import fr.radi3nt.physics.core.TransformedObject;

import java.util.Collection;

public class NormalSatCollisionDetector implements CollisionDetector {

    private static final ObjectPool<Vector3f> VECTOR_3_F_OBJECT_POOL = new Vector3fPool(20);

    private static final float EPSILON = 1e-5f;
    private Vector3f mta = null;
    private final Vector3f mtaResult = new SimpleVector3f();
    private float minOverlap;
    private int normal;

    private Vector3f[] verticesA;
    private Vector3f[] verticesB;

    private SatShapeObject shape1;
    private SatShapeObject shape2;

    private final SatProjectedObject p1 = new SatProjectedObject(0, 0);
    private final SatProjectedObject p2 = new SatProjectedObject(0, 0);

    public boolean testCollision(ContactPair pair, SatShapeObject shape1, SatShapeObject shape2) {
        this.shape1 = shape1;
        this.shape2 = shape2;
        mta = null;
        minOverlap = Float.MAX_VALUE;
        normal = 1;

        Vector3f[] shape1AxisArray = getNormalTransformedArray(shape1.getAxis(), pair.objectA);
        Vector3f[] shape2AxisArray = getNormalTransformedArray(shape2.getAxis(), pair.objectB);

        computeVertices(pair, shape1, shape2);

        /*
        for (Vector3f axisA : shape1AxisArray) {
            boolean isSame = false;
            for (Vector3f axisB : shape2AxisArray) {
                float dot = axisA.dot(axisB);
                if (dot>=1) {
                    isSame = true;
                    break;
                }
            }
            if (!isSame)
                if (axisHadNoOverlap(axisA))
                    return false;
        }
         */

            boolean existingSeparatingAxisForShape1 = processCollision(shape1AxisArray);
            if (existingSeparatingAxisForShape1) {
                freeAll(shape1AxisArray, shape2AxisArray);
                return false;
            }

            boolean existingSeparatingAxisForShape2 = processCollision(shape2AxisArray);
            if (existingSeparatingAxisForShape2) {
                freeAll(shape1AxisArray, shape2AxisArray);
                return false;
            }

            boolean existingSeparatingAxisForEdges = processCollisionEdges(pair, shape1, shape2);
            if (existingSeparatingAxisForEdges) {
                freeAll(shape1AxisArray, shape2AxisArray);
                return false;
            }

        mta = mta.duplicate();
        freeAll(shape1AxisArray, shape2AxisArray);

        return true;
    }

    private void freeAll(Vector3f[] shape1AxisArray, Vector3f[] shape2AxisArray) {
        freeArray(shape2AxisArray);
        freeArray(shape1AxisArray);
        freeArray(verticesA);
        freeArray(verticesB);
    }

    private static void freeArray(Vector3f[] shape1AxisArray) {
        if (shape1AxisArray==null)
            return;
        for (Vector3f vector3f : shape1AxisArray) {
            VECTOR_3_F_OBJECT_POOL.free(vector3f);
        }
    }

    public void computeVertices(ContactPair pair, SatShapeObject shape1, SatShapeObject shape2) {
        verticesA = getTransformedArray(shape1.getVertices(), pair.objectA);
        verticesB = getTransformedArray(shape2.getVertices(), pair.objectB);
    }

    public static Vector3f[] getTransformedArray(Vector3f[] originalArray, TransformedObject object) {
        Vector3f[] shapeAxisArray = new Vector3f[originalArray.length];
        for (int i = 0; i < originalArray.length; i++) {
            shapeAxisArray[i] = object.toWorldSpace(originalArray[i], VECTOR_3_F_OBJECT_POOL.borrow());
        }
        return shapeAxisArray;
    }

    public static Vector3f[] getNormalTransformedArray(Vector3f[] originalArray, TransformedObject object) {
        Vector3f[] shapeAxisArray = new Vector3f[originalArray.length];
        for (int i = 0; i < originalArray.length; i++) {
            Vector3f vertex = VECTOR_3_F_OBJECT_POOL.borrow();
            vertex.copy(originalArray[i]);
            object.getRotation().transform(vertex);
            shapeAxisArray[i] = vertex;
        }
        return shapeAxisArray;
    }

    public Vector3f getMinimumTranslationAxis() {
        return mta;
    }

    public float getCurrentOverlap() {
        return minOverlap;
    }

    public int getOverlapNormal() {
        return normal;
    }

    private boolean processCollisionEdges(ContactPair pair, SatShapeObject shape1, SatShapeObject shape2) {
        Vector3f[] shape1Edges = shape1.getEdges();
        Vector3f[] shape2Edges = shape2.getEdges();

        for (Vector3f shape1Edge : shape1Edges) {
            Vector3f edge1 = shape1Edge.duplicate();
            pair.objectA.getRotation().transform(edge1);

            for (Vector3f shape2Edge : shape2Edges) {
                Vector3f edge2 = shape2Edge.duplicate();
                pair.objectB.getRotation().transform(edge2);

                Vector3f satAxis = edge2.cross(edge1);
                if (satAxis.lengthSquared() < EPSILON || Double.isNaN(satAxis.lengthSquared()))
                    continue;

                satAxis.normalize();

                if (axisHadNoOverlap(satAxis))
                    return true;
            }
        }

        return false;
    }

    private boolean processCollision(Collection<Vector3f> axisArray) {
        for (Vector3f axis : axisArray) {
            if (axisHadNoOverlap(axis))
                return true;
        }

        return false;
    }

    private boolean processCollision(Vector3f[] axisArray) {
        for (Vector3f axis : axisArray) {
            if (axisHadNoOverlap(axis))
                return true;
        }

        return false;
    }

    private boolean axisHadNoOverlap(Vector3f axis) {
        SatProjectedObject p1 = this.p1.projectReplace(verticesA, axis);
        SatProjectedObject p2 = this.p2.projectReplace(verticesB, axis);

        float overlap = SatProjectedObject.getOverlap(p1, p2);
        if (overlap > 0) {
            if (overlap < minOverlap) {
                int expectedNormal = SatProjectedObject.getOverlapNormal(p1, p2);

                minOverlap = overlap;
                mta = axis;
                normal = expectedNormal;

                mtaResult.copy(mta);
                normal = shape1.transformNormal(mtaResult, normal);
                normal = shape2.transformNormal(mtaResult, normal);
                mta = mtaResult;
            }
            return false;
        }

        return true;
    }
}
