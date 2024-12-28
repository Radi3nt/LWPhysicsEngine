package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.computer.detector.sweep;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.maths.pool.ObjectPool;
import fr.radi3nt.maths.pool.Vector3fPool;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.computer.detector.NormalSatCollisionDetector;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.info.SatAxis;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.info.SatEdge;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.projection.SatProjectedObject;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.projection.SatProjectionProvider;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.simple.SatShape;
import fr.radi3nt.physics.core.TransformedObject;

import java.util.ArrayList;
import java.util.Collection;

import static java.lang.Math.abs;

public class SweepSatCollisionDetector {

    public static final ObjectPool<Vector3f> VECTOR_3_F_OBJECT_POOL = new Vector3fPool(20);
    private static final float EPSILON = 1e-5f;

    private final SatProjectedObject p1 = new SatProjectedObject(0, 0);
    private final SatProjectedObject p2 = new SatProjectedObject(0, 0);

    private SatProjectionProvider providerA;
    private SatProjectionProvider providerB;

    private SweepInfoProvider provider;

    private NormalSatCollisionDetector.MinimumInfo minimumInfo;
    private NormalSatCollisionDetector.SatContactType currentContactType;
    private boolean edgeAllowedAsMinimum = true;

    private float collisionTEnter = Float.NEGATIVE_INFINITY;
    private float collisionTLeave = Float.POSITIVE_INFINITY;

    private ResultInfo resultInfo;

    public boolean testCollision(TransformedObject a, TransformedObject b, SatShape shape1, SatShape shape2) {
        minimumInfo = new NormalSatCollisionDetector.MinimumInfo();
        collisionTEnter = Float.NEGATIVE_INFINITY;
        collisionTLeave = Float.POSITIVE_INFINITY;

        SatAxis[] shapeAAxis = shape1.getSatAxis();
        SatAxis[] shapeBAxis = shape2.getSatAxis();

        edgeAllowedAsMinimum = shape1.canUseEdgesAsMinimum() && shape2.canUseEdgesAsMinimum();

        computeVertices(a, b, shape1, shape2);

        currentContactType = NormalSatCollisionDetector.SatContactType.A_AXIS;

        if (separatingAxisFound(shapeAAxis, a)) {
            freeAll();
            return false;
        }

        currentContactType = NormalSatCollisionDetector.SatContactType.B_AXIS;

        if (separatingAxisFound(shapeBAxis, b)) {
            freeAll();
            return false;
        }

        currentContactType = NormalSatCollisionDetector.SatContactType.EDGE_AXIS;

        Collection<NormalSatCollisionDetector.EdgeCross> edgeCrosses = new ArrayList<>();
        if (separatingCollisionEdgeFound(a, b, shape1, shape2, edgeCrosses)) {
            freeAll();
            return false;
        }

        freeAll();

        if (Float.isNaN(collisionTLeave)) {
            return false;
        }

        correctNormal(shape1, shape2, minimumInfo.type);

        resultInfo = new ResultInfo(minimumInfo, collisionTEnter, collisionTLeave);

        NormalSatCollisionDetector.SatContactType contactType = minimumInfo.type;
        if (minimumInfo.type.isFromA()) {
            resultInfo.originateFromA = true;
            resultInfo.aAxis = minimumInfo.foundAxis;
            for (SatAxis axis : shapeBAxis) {
                Vector3f transformedAxis = axis.getAxis().duplicate();
                b.getRotation().transform(transformedAxis);
                if (abs(transformedAxis.dot(minimumInfo.worldSpaceAxis))!=1)
                    continue;
                resultInfo.bAxis = axis;
                contactType = NormalSatCollisionDetector.SatContactType.AB_AXIS;
            }
        }
        if (minimumInfo.type.isFromB()) {
            resultInfo.originateFromA = false;
            resultInfo.bAxis = minimumInfo.foundAxis;
            for (SatAxis axis : shapeAAxis) {
                Vector3f transformedAxis = axis.getAxis().duplicate();
                a.getRotation().transform(transformedAxis);
                if (abs(transformedAxis.dot(minimumInfo.worldSpaceAxis))!=1)
                    continue;
                resultInfo.aAxis = axis;
                contactType = NormalSatCollisionDetector.SatContactType.AB_AXIS;
            }
        }
        if (!minimumInfo.type.isFace()) {
            Vector3f retainedCross = minimumInfo.edgeCross.crossed;
            resultInfo.edgeCrosses.add(minimumInfo.edgeCross);
            for (NormalSatCollisionDetector.EdgeCross edgeCross : edgeCrosses) {
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

    public void setProvider(SweepInfoProvider provider) {
        this.provider = provider;
    }

    public ResultInfo getResultInfo() {
        return resultInfo;
    }

    private void correctNormal(SatShape shape1, SatShape shape2, NormalSatCollisionDetector.SatContactType contactType) {
        int expectedNormal = minimumInfo.normal;
        expectedNormal = shape1.transformNormal(minimumInfo.worldSpaceAxis, expectedNormal, true, contactType);
        expectedNormal = shape2.transformNormal(minimumInfo.worldSpaceAxis, expectedNormal, false, contactType);
        minimumInfo.normal = expectedNormal;
    }

    private void freeAll() {
        providerA.free(VECTOR_3_F_OBJECT_POOL);
        providerB.free(VECTOR_3_F_OBJECT_POOL);
    }

    public void computeVertices(TransformedObject a, TransformedObject b, SatShape shape1, SatShape shape2) {
        providerA = shape1.getSatProjectionProvider(a, VECTOR_3_F_OBJECT_POOL);
        providerB = shape2.getSatProjectionProvider(b, VECTOR_3_F_OBJECT_POOL);
    }

    private boolean separatingCollisionEdgeFound(TransformedObject a, TransformedObject b, SatShape shape1, SatShape shape2, Collection<NormalSatCollisionDetector.EdgeCross> edgeCrosses) {
        SatEdge[] shape1Edges = shape1.getSatEdges();
        SatEdge[] shape2Edges = shape2.getSatEdges();

        Collection<Vector3f> uniqueEdgesCrosses = new ArrayList<>();

        for (SatEdge shape1Edge : shape1Edges) {
            Vector3f edge1 = shape1Edge.getAxis().duplicate();
            a.getRotation().transform(edge1);

            for (SatEdge shape2Edge : shape2Edges) {
                Vector3f edge2 = shape2Edge.getAxis().duplicate();
                b.getRotation().transform(edge2);

                Vector3f satAxis = edge2.cross(edge1);
                if (axisWereCollinear(satAxis))
                    continue;

                satAxis.normalize();
                NormalSatCollisionDetector.EdgeCross edgeCross = new NormalSatCollisionDetector.EdgeCross(shape1Edge, shape2Edge, satAxis);
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

    private static boolean axisWereCollinear(Vector3f satAxis) {
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

    private boolean axisHadNoOverlap(Vector3f axis, NormalSatCollisionDetector.EdgeCross edgeCross) {
        float relativeSpeed = provider.getRelativeVelocityOnAxis(axis);
        providerA.project(p1, axis);
        providerB.project(p2, axis);

        float enterTime = Float.NEGATIVE_INFINITY;
        float leaveTime = Float.POSITIVE_INFINITY;
        if (abs(relativeSpeed) < EPSILON) {
            if (SatProjectedObject.getOverlap(p1, p2)==Float.NEGATIVE_INFINITY) {
                this.collisionTEnter = Float.POSITIVE_INFINITY;
                this.collisionTLeave = Float.NEGATIVE_INFINITY;
                this.minimumInfo.type = currentContactType;
                return true;
            }
        } else {
            enterTime = SatProjectedObject.getEnterTime(p1, p2, relativeSpeed);
            leaveTime = SatProjectedObject.getLeaveTime(p1, p2, relativeSpeed);
        }

        this.collisionTLeave = Math.min(this.collisionTLeave, leaveTime);

        boolean newEnter = enterTime >= this.collisionTEnter;
        if (newEnter) {
            this.collisionTEnter = enterTime;
        }

        if (provider.noCollision(this.collisionTEnter, this.collisionTLeave)) {
            return true;
        }

        if (newEnter) {
            if (!edgeAllowedAsMinimum)
                return false;

            updateMinimum(axis);
            minimumInfo.foundAxis = null;
            minimumInfo.edgeCross = edgeCross;
        }

        return false;
    }

    private boolean faceAxisHadNoOverlap(Vector3f axis, SatAxis satAxis) {
        float relativeSpeed = provider.getRelativeVelocityOnAxis(axis);
        providerA.project(p1, axis);
        providerB.project(p2, axis);

        float enterTime = Float.NEGATIVE_INFINITY;
        float leaveTime = Float.POSITIVE_INFINITY;
        if (abs(relativeSpeed) < EPSILON) {
            if (SatProjectedObject.getOverlap(p1, p2)==Float.NEGATIVE_INFINITY) {
                this.collisionTEnter = Float.POSITIVE_INFINITY;
                this.collisionTLeave = Float.NEGATIVE_INFINITY;
                this.minimumInfo.type = currentContactType;
                return true;
            }
        } else {
            enterTime = SatProjectedObject.getEnterTime(p1, p2, relativeSpeed);
            leaveTime = SatProjectedObject.getLeaveTime(p1, p2, relativeSpeed);
        }

        this.collisionTLeave = Math.min(this.collisionTLeave, leaveTime);

        boolean newEnter = enterTime >= this.collisionTEnter;
        if (newEnter) {
            this.collisionTEnter = enterTime;
        }

        if (provider.noCollision(this.collisionTEnter, this.collisionTLeave)) {
            return true;
        }

        if (newEnter) {
            updateMinimum(axis);
            minimumInfo.foundAxis = satAxis;
            minimumInfo.edgeCross = null;
        }

        return false;
    }

    private void updateMinimum(Vector3f axis) {
        int expectedNormal = SatProjectedObject.getOverlapNormal(p1, p2);

        minimumInfo.worldSpaceAxis.copy(axis);
        minimumInfo.normal = expectedNormal;
        minimumInfo.type = currentContactType;
    }

    public static class ResultInfo extends NormalSatCollisionDetector.ResultInfo {

        public final float tEnter;
        public final float tLeave;

        public ResultInfo(NormalSatCollisionDetector.MinimumInfo minimumInfo, float tEnter, float tLeave) {
            super(minimumInfo);
            this.tEnter = tEnter;
            this.tLeave = tLeave;

        }
    }

}
