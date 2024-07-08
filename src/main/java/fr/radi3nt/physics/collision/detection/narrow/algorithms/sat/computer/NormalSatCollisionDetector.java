package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.computer;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.maths.pool.ObjectPool;
import fr.radi3nt.maths.pool.Vector3fPool;
import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.info.SatAxis;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.info.SatEdge;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.projection.SatProjectedObject;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.projection.SatProjectionProvider;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape.SatProcessedShape;
import fr.radi3nt.physics.core.TransformedObject;

import java.util.ArrayList;
import java.util.Collection;

import static java.lang.Math.abs;

public class NormalSatCollisionDetector implements SatCollisionDetector {

    public static final ObjectPool<Vector3f> VECTOR_3_F_OBJECT_POOL = new Vector3fPool(20);
    private static final float EPSILON = 1e-5f;

    private final SatProjectedObject p1 = new SatProjectedObject(0, 0);
    private final SatProjectedObject p2 = new SatProjectedObject(0, 0);

    private SatProjectionProvider providerA;
    private SatProjectionProvider providerB;

    private MinimumInfo minimumInfo;
    private SatContactType currentContactType;
    private boolean edgeAllowedAsMinimum = true;

    private ResultInfo resultInfo;

    public boolean testCollision(GeneratedContactPair pair, SatProcessedShape shape1, SatProcessedShape shape2) {

        minimumInfo = new MinimumInfo();

        SatAxis[] shapeAAxis = shape1.getSatAxis();
        SatAxis[] shapeBAxis = shape2.getSatAxis();

        edgeAllowedAsMinimum = shape1.canUseEdgesAsMinimum() && shape2.canUseEdgesAsMinimum();

        computeVertices(pair, shape1, shape2);

        currentContactType = SatContactType.A_AXIS;

        if (separatingAxisFound(shapeAAxis, pair.objectA)) {
            freeAll();
            return false;
        }

        currentContactType = SatContactType.B_AXIS;

        if (separatingAxisFound(shapeBAxis, pair.objectB)) {
            freeAll();
            return false;
        }

        currentContactType = SatContactType.EDGE_AXIS;

        Collection<EdgeCross> edgeCrosses = new ArrayList<>();
        if (separatingCollisionEdgeFound(pair, shape1, shape2, edgeCrosses)) {
            freeAll();
            return false;
        }

        freeAll();

        correctNormal(shape1, shape2, minimumInfo.type);

        resultInfo = new ResultInfo(minimumInfo);

        SatContactType contactType = minimumInfo.type;
        if (minimumInfo.type.isFromA()) {
            resultInfo.originateFromA = true;
            resultInfo.aAxis = minimumInfo.foundAxis;
            for (SatAxis axis : shapeBAxis) {
                Vector3f transformedAxis = axis.getAxis().duplicate();
                pair.objectB.getRotation().transform(transformedAxis);
                if (abs(transformedAxis.dot(minimumInfo.worldSpaceAxis))!=1)
                    continue;
                resultInfo.bAxis = axis;
                contactType = SatContactType.AB_AXIS;
            }
        }
        if (minimumInfo.type.isFromB()) {
            resultInfo.originateFromA = false;
            resultInfo.bAxis = minimumInfo.foundAxis;
            for (SatAxis axis : shapeAAxis) {
                Vector3f transformedAxis = axis.getAxis().duplicate();
                pair.objectA.getRotation().transform(transformedAxis);
                if (abs(transformedAxis.dot(minimumInfo.worldSpaceAxis))!=1)
                    continue;
                resultInfo.aAxis = axis;
                contactType = SatContactType.AB_AXIS;
            }
        }
        if (!minimumInfo.type.isFace()) {
            Vector3f retainedCross = minimumInfo.edgeCross.crossed;
            resultInfo.edgeCrosses.add(minimumInfo.edgeCross);
            for (EdgeCross edgeCross : edgeCrosses) {
                if (edgeCross==minimumInfo.edgeCross)
                    continue;
                if (abs(abs(edgeCross.crossed.dot(retainedCross))-1)>1e-1f)
                    continue;
                resultInfo.edgeCrosses.add(edgeCross);
            }
        }

        resultInfo.contactType = contactType;

        return true;
    }

    public ResultInfo getResultInfo() {
        return resultInfo;
    }

    private void correctNormal(SatProcessedShape shape1, SatProcessedShape shape2, SatContactType contactType) {
        int expectedNormal = minimumInfo.normal;
        expectedNormal = shape1.transformNormal(minimumInfo.worldSpaceAxis, expectedNormal, true, contactType);
        expectedNormal = shape2.transformNormal(minimumInfo.worldSpaceAxis, expectedNormal, false, contactType);
        minimumInfo.normal = expectedNormal;
    }

    private void freeAll() {
        providerA.free(VECTOR_3_F_OBJECT_POOL);
        providerB.free(VECTOR_3_F_OBJECT_POOL);
    }

    private static void freeArray(Vector3f[] shape1AxisArray) {
        if (shape1AxisArray==null)
            return;
        for (Vector3f vector3f : shape1AxisArray) {
            VECTOR_3_F_OBJECT_POOL.free(vector3f);
        }
    }

    public void computeVertices(GeneratedContactPair pair, SatProcessedShape shape1, SatProcessedShape shape2) {
        providerA = shape1.getSatProjectionProvider(pair.objectA, VECTOR_3_F_OBJECT_POOL);
        providerB = shape2.getSatProjectionProvider(pair.objectB, VECTOR_3_F_OBJECT_POOL);
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

    public static Vector3f[] getNormalTransformedArray(SatAxis[] originalArray, TransformedObject object) {
        Vector3f[] shapeAxisArray = new Vector3f[originalArray.length];
        for (int i = 0; i < originalArray.length; i++) {
            Vector3f vertex = VECTOR_3_F_OBJECT_POOL.borrow();
            vertex.copy(originalArray[i].getAxis());
            object.getRotation().transform(vertex);
            shapeAxisArray[i] = vertex;
        }
        return shapeAxisArray;
    }

    private boolean separatingCollisionEdgeFound(GeneratedContactPair pair, SatProcessedShape shape1, SatProcessedShape shape2, Collection<EdgeCross> edgeCrosses) {
        SatEdge[] shape1Edges = shape1.getSatEdges();
        SatEdge[] shape2Edges = shape2.getSatEdges();

        Collection<Vector3f> uniqueEdgesCrosses = new ArrayList<>();

        for (SatEdge shape1Edge : shape1Edges) {
            Vector3f edge1 = shape1Edge.getAxis().duplicate();
            pair.objectA.getRotation().transform(edge1);

            for (SatEdge shape2Edge : shape2Edges) {
                Vector3f edge2 = shape2Edge.getAxis().duplicate();
                pair.objectB.getRotation().transform(edge2);

                Vector3f satAxis = edge2.cross(edge1);
                if (axisWereColinear(satAxis))
                    continue;

                satAxis.normalize();
                EdgeCross edgeCross = new EdgeCross(shape1Edge, shape2Edge, satAxis);
                edgeCrosses.add(edgeCross);

                boolean alreadyTestedAxis = alreadyTested(uniqueEdgesCrosses, satAxis);
                if (alreadyTestedAxis)
                    continue;
                uniqueEdgesCrosses.add(satAxis);

                if (axisHadNoOverlap(satAxis, edgeCross))
                    return true;
            }
        }

        return false;
    }

    private boolean alreadyTested(Collection<Vector3f> edgeCrosses, Vector3f satAxis) {
        for (Vector3f edgeCross : edgeCrosses) {
            if (abs(edgeCross.dot(satAxis))!=1)
                continue;
            return true;
        }
        return false;
    }

    private static boolean axisWereColinear(Vector3f satAxis) {
        return satAxis.lengthSquared() < EPSILON || Double.isNaN(satAxis.lengthSquared());
    }

    private boolean separatingAxisFound(SatAxis[] shapeAxis, TransformedObject object) {
        Vector3f transformedAxis = new SimpleVector3f();
        for (SatAxis current : shapeAxis) {
            Vector3f axis = current.getAxis();
            transformedAxis.copy(axis);
            object.getRotation().transform(transformedAxis);

            if (faceAxisHadNoOverlap(transformedAxis, current)) {
                return true;
            }
        }

        return false;
    }

    private boolean axisHadNoOverlap(Vector3f axis, EdgeCross edgeCross) {
        float shortestDistance = getShortestDistance(axis);
        if (isNotPenetration(shortestDistance))
            return true;

        float overlap = -shortestDistance;
        if (isNotNewMinimum(overlap)) {
            return false;
        }
        if (!edgeAllowedAsMinimum)
            return false;

        updateMinimum(axis, overlap);
        minimumInfo.foundAxis = null;
        minimumInfo.edgeCross = edgeCross;
        return false;
    }

    private boolean faceAxisHadNoOverlap(Vector3f axis, SatAxis satAxis) {
        float shortestDistance = getShortestDistance(axis);
        if (isNotPenetration(shortestDistance))
            return true;

        float overlap = -shortestDistance;
        if (isNotNewMinimum(overlap)) {
            return false;
        }

        updateMinimum(axis, overlap);
        minimumInfo.foundAxis = satAxis;
        minimumInfo.edgeCross = null;
        return false;
    }

    private static boolean isNotPenetration(float shortestDistance) {
        return shortestDistance >= 0;
    }

    private boolean isNotNewMinimum(float overlap) {
        return overlap >= minimumInfo.overlap;
    }

    private void updateMinimum(Vector3f axis, float overlap) {
        int expectedNormal = SatProjectedObject.getOverlapNormal(p1, p2);

        minimumInfo.worldSpaceAxis.copy(axis);
        minimumInfo.normal = expectedNormal;
        minimumInfo.overlap = overlap;
        minimumInfo.type = currentContactType;
    }
    private float getShortestDistance(Vector3f axis) {
        providerA.project(p1, axis);
        providerB.project(p2, axis);

        return SatProjectedObject.getShortestDistance(p1, p2);
    }

    public static class MinimumInfo {

        public Vector3f worldSpaceAxis = new SimpleVector3f();
        public int normal;

        public float overlap = Float.MAX_VALUE;
        public SatContactType type;

        public SatAxis foundAxis;
        public EdgeCross edgeCross;
    }

    public static class ResultInfo {

        public final Vector3f worldSpaceAxis;
        public final Vector3f worldSpaceNormal;
        public final int normal;

        public final float overlap;

        public SatContactType contactType;
        public SatAxis aAxis;
        public SatAxis bAxis;
        public Collection<EdgeCross> edgeCrosses = new ArrayList<>();
        public boolean originateFromA;


        public ResultInfo(MinimumInfo minimumInfo) {
            this.worldSpaceAxis = minimumInfo.worldSpaceAxis;
            this.normal = minimumInfo.normal;
            this.overlap = abs(minimumInfo.overlap);

            worldSpaceNormal = worldSpaceAxis.duplicate().mul(normal);
        }
    }

    public enum SatContactType {

        A_AXIS(true, true, false),
        B_AXIS(true, false, true),
        AB_AXIS(true, true, true),
        EDGE_AXIS(false, false, false),
        ;

        private final boolean face;
        private final boolean fromA;
        private final boolean fromB;

        SatContactType(boolean face, boolean fromA, boolean fromB) {
            this.face = face;
            this.fromA = fromA;
            this.fromB = fromB;
        }

        public boolean isFace() {
            return face;
        }

        public boolean isFromA() {
            return fromA;
        }

        public boolean isFromB() {
            return fromB;
        }
    }

    public static class EdgeCross {

        public final SatEdge aEdge;
        public final SatEdge bEdge;
        public final Vector3f crossed;

        public EdgeCross(SatEdge aEdge, SatEdge bEdge, Vector3f crossed) {
            this.aEdge = aEdge;
            this.bEdge = bEdge;
            this.crossed = crossed;
        }
    }
}
