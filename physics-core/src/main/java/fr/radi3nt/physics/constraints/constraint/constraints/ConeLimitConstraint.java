package fr.radi3nt.physics.constraints.constraint.constraints;

import fr.radi3nt.maths.components.advanced.matrix.angle.Angle;
import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.arbitrary.VectorNf;
import fr.radi3nt.maths.components.arbitrary.vector.ArrayVectorNf;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.constraints.constraint.Constraint;
import fr.radi3nt.physics.constraints.constraint.ConstraintData;
import fr.radi3nt.physics.constraints.constraint.DriftParameters;
import fr.radi3nt.physics.constraints.constraint.StateConstraint;
import fr.radi3nt.physics.constraints.constraint.caching.provider.CachingModuleProvider;
import fr.radi3nt.physics.constraints.constraint.index.IdentifiedDynamicsData;
import fr.radi3nt.physics.constraints.constraint.index.RigidBodyIndex;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

import java.util.ArrayList;
import java.util.List;

public class ConeLimitConstraint implements Constraint {

    private final RigidBodyIndex indexA;
    private final RigidBodyIndex indexB;

    private final Quaternion initialDifference;
    private final Quaternion initialA;
    private final Quaternion initialB;
    private final Angle angle;
    private final DriftParameters driftParameters;
    private final CachingModuleProvider provider;
    private final float limit;

    public ConeLimitConstraint(RigidBodyIndex indexA, RigidBodyIndex indexB, Quaternion rotationA, Quaternion rotationB, Angle angle, DriftParameters driftParameters, CachingModuleProvider provider, float limit) {
        this.indexA = indexA;
        this.indexB = indexB;
        this.initialDifference = differenceQuaternion(rotationB, rotationA);
        this.initialA = rotationA;
        this.initialB = rotationB;
        this.angle = angle;
        this.driftParameters = driftParameters;
        this.provider = provider;
        this.limit = limit;
    }

    private static Quaternion differenceQuaternion(Quaternion rotationA, Quaternion rotationB) {
        Quaternion invA = rotationA.duplicate();
        invA.inverse();
        Quaternion result = rotationB.duplicate();

        result.multiply(invA);

        return result;
    }

    @Override
    public ConstraintData[] compute(RigidBodyIsland island) {
        IdentifiedDynamicsData indexA = this.indexA.getIdentifiedData(island);
        IdentifiedDynamicsData indexB = this.indexB.getIdentifiedData(island);


        Quaternion currentDiff = calculateDiff(indexA, indexB);


        List<ConstraintData> data = new ArrayList<>();

        correctDiff(currentDiff, data, indexA, indexB);

        return data.toArray(new ConstraintData[0]);
    }

    private Quaternion calculateDiff(IdentifiedDynamicsData indexA, IdentifiedDynamicsData indexB) {

        Quaternion rotToApply = differenceQuaternion(initialA, indexA.data.getRotation());
        Quaternion resultRot = rotToApply.duplicate();
        resultRot.multiply(initialB);
        Quaternion currentDiff = differenceQuaternion(resultRot, indexB.data.getRotation());

        return currentDiff;
    }

    private Object toReadableQuat(Quaternion current) {
        Vector3f axis = current.getAxisOrDefault(new SimpleVector3f());
        return "Rotation: {" + axis.getX() + ", " + axis.getY() + ", " + axis.getZ() + "} / (" + current.getAngle().getDegree() + "°)";
    }

    protected void correctDiff(Quaternion currentDiff, List<ConstraintData> data, IdentifiedDynamicsData indexA, IdentifiedDynamicsData indexB) {
        applyCorrection(data, indexA, indexB, (float) currentDiff.getAngle().getRadiant(), currentDiff.getAxisOrDefault(new SimpleVector3f(1, 0, 0)).normalizeSafely());
    }

    protected void applyCorrection(List<ConstraintData> data, IdentifiedDynamicsData indexA, IdentifiedDynamicsData indexB, float currentAngle, Vector3f constrainedAxis) {
        Vector3f axis = constrainedAxis.duplicate();

        if (currentAngle>Math.PI) {
            currentAngle -= (float) (Math.PI*2);
        }

        if (currentAngle<0) {
            currentAngle*=-1;
            axis.negate();
        }

        float limitAngle = (float) angle.getRadiant();

        float correctionMin = limitAngle-currentAngle;

        if (correctionMin>0)
            return;
        data.add(new AxisLimitConstraintData(indexA, indexB, axis.duplicate().negate(), correctionMin));
    }

    @Override
    public RigidBodyIndex[] getConcernedBodies() {
        return new RigidBodyIndex[] {indexA, indexB};
    }

    protected class AxisLimitConstraintData implements ConstraintData {

        private final IdentifiedDynamicsData indexA;
        private final IdentifiedDynamicsData indexB;
        private final Vector3f constrainedAxisA;
        private final Vector3f constrainedAxisB;

        private final float correction;

        private AxisLimitConstraintData(IdentifiedDynamicsData indexA, IdentifiedDynamicsData indexB, Vector3f constrainedAxis, float correction) {
            this.indexA = indexA;
            this.indexB = indexB;

            this.correction = correction;

            this.constrainedAxisA = constrainedAxis.duplicate();
            this.constrainedAxisB = constrainedAxis.duplicate();
        }

        @Override
        public float[] getMin() {
            return new float[] {0};
        }

        @Override
        public float[] getMax() {
            return new float[] {limit};
        }

        @Override
        public float[] getCorrections() {
            return new float[] {correction};
        }

        @Override
        public StateConstraint[] getImpulses() {
            return new StateConstraint[] {
                new StateConstraint(new IdentifiedDynamicsData[] {indexA, indexB}, new VectorNf[]{
                        new ArrayVectorNf(0, 0, 0, -constrainedAxisA.getX(), -constrainedAxisA.getY(), -constrainedAxisA.getZ()),
                        new ArrayVectorNf(0, 0, 0, constrainedAxisB.getX(), constrainedAxisB.getY(), constrainedAxisB.getZ())
                }),
            };
        }

        @Override
        public DriftParameters[] getDriftParameters() {
            return new DriftParameters[] {driftParameters};
        }

        @Override
        public CachingModuleProvider getCachingConstraintModules() {
            return provider;
        }
    }
}
