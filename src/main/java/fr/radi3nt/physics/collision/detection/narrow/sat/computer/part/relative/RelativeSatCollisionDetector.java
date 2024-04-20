package fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.relative;

import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.contact.ContactPair;
import fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.CollisionDetector;
import fr.radi3nt.physics.collision.shape.sat.SatProjectedObject;
import fr.radi3nt.physics.collision.shape.sat.shape.SatShapeObject;

import java.util.Collection;

import static java.lang.Math.abs;

public class RelativeSatCollisionDetector implements CollisionDetector {

    private static final float EPSILON = 1e-5f;
    private Vector3f mta = null;
    private Vector3f worldMta = null;
    private float minOverlap;
    private int normal;

    private Quaternion relativeRotation;
    private Vector3f relativePosition;

    private Vector3f[] verticesA;
    private Vector3f[] verticesB;

    public boolean testCollision(ContactPair pair, SatShapeObject shape1, SatShapeObject shape2) {
        mta = null;
        minOverlap = Float.MAX_VALUE;
        normal = 1;

        Quaternion inverseA = pair.objectA.getRotation().duplicate();
        inverseA.inverse();

        relativeRotation = pair.objectB.getRotation().duplicate();
        relativeRotation.multiply(inverseA);

        Vector3f[] shape1AxisArray = shape1.getAxis();
        Vector3f[] shape2AxisArray = getNormalTransformedArray(shape2.getAxis(), relativeRotation);

        relativePosition = pair.objectB.getPosition().duplicate().sub(pair.objectA.getPosition());
        inverseA.transform(relativePosition);

        verticesA = shape1.getVertices();
        verticesB = getTransformedArray(shape2.getVertices(), relativeRotation, relativePosition);

        for (Vector3f axisA : shape1AxisArray) {
            boolean isSame = false;
            for (Vector3f axisB : shape2AxisArray) {
                float dot = axisA.dot(axisB);
                if (abs(dot)>=1f-EPSILON) {
                    isSame = true;
                    break;
                }
            }
            if (!isSame)
                if (axisHadNoOverlap(axisA))
                    return false;
        }

        boolean existingSeparatingAxisForShape2 = processCollision(shape2AxisArray);
        if (existingSeparatingAxisForShape2)
            return false;
        boolean existingSeparatingAxisForEdges = processCollisionEdges(shape1, shape2, relativeRotation);
        if (existingSeparatingAxisForEdges)
            return false;

        worldMta = mta.duplicate();
        pair.objectA.getRotation().transform(worldMta);

        return true;
    }

    private static Vector3f[] getTransformedArray(Vector3f[] originalArray, Quaternion rotation, Vector3f relativePosition) {
        Vector3f[] shapeAxisArray = new Vector3f[originalArray.length];
        for (int i = 0; i < originalArray.length; i++) {
            Vector3f vertex = originalArray[i].duplicate();
            rotation.transform(vertex);
            vertex.add(relativePosition);
            shapeAxisArray[i] = vertex;
        }
        return shapeAxisArray;
    }

    private static Vector3f[] getNormalTransformedArray(Vector3f[] originalArray, Quaternion rotation) {
        Vector3f[] shapeAxisArray = new Vector3f[originalArray.length];
        for (int i = 0; i < originalArray.length; i++) {
            Vector3f vertex = originalArray[i].duplicate();
            rotation.transform(vertex);
            shapeAxisArray[i] = vertex;
        }
        return shapeAxisArray;
    }

    public Vector3f getMinimumTranslationAxis() {
        return worldMta;
    }

    public Vector3f getRelativeMinimumTranslationAxis() {
        return mta;
    }

    public float getCurrentOverlap() {
        return minOverlap;
    }

    public int getOverlapNormal() {
        return normal;
    }

    private boolean processCollisionEdges(SatShapeObject shape1, SatShapeObject shape2, Quaternion relativeRot) {
        Vector3f[] shape1Edges = shape1.getEdges();
        Vector3f[] shape2Edges = shape2.getEdges();

        for (Vector3f shape1Edge : shape1Edges) {
            Vector3f edge1 = shape1Edge.duplicate();

            for (Vector3f shapeEdge2 : shape2Edges) {
                Vector3f edge2 = shapeEdge2.duplicate();
                relativeRot.transform(edge2);

                Vector3f satAxis = edge1.cross(edge2);
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
        SatProjectedObject p1 = SatProjectedObject.project(verticesA, axis);
        SatProjectedObject p2 = SatProjectedObject.project(verticesB, axis);

        float overlap = SatProjectedObject.getOverlap(p1, p2);
        if (overlap > 0) {
            updateCurrentOverlap(axis, overlap, p1, p2);
            return false;
        }

        return true;
    }

    private void updateCurrentOverlap(Vector3f axis, float overlap, SatProjectedObject p1, SatProjectedObject p2) {
        if (overlap < minOverlap) {
            minOverlap = overlap;
            mta = axis;
            normal = SatProjectedObject.getOverlapNormal(p1, p2);
        }
    }

    public Vector3f[] getLocalVerticesA() {
        return verticesA;
    }

    public Vector3f[] getRelativeVerticesB() {
        return verticesB;
    }

    public Quaternion getRelativeRotation() {
        return relativeRotation;
    }

    public Vector3f getRelativePosition() {
        return relativePosition;
    }
}
