package fr.radi3nt.physics.constraints.constraint.constraints;

import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.arbitrary.VectorNf;
import fr.radi3nt.maths.components.arbitrary.vector.ArrayVectorNf;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.constraints.constraint.Constraint;
import fr.radi3nt.physics.constraints.constraint.ConstraintData;
import fr.radi3nt.physics.constraints.constraint.DriftParameters;
import fr.radi3nt.physics.constraints.constraint.StateConstraint;
import fr.radi3nt.physics.constraints.constraint.caching.provider.CachingModuleProvider;
import fr.radi3nt.physics.constraints.constraint.index.IdentifiedDynamicsData;
import fr.radi3nt.physics.constraints.constraint.index.RigidBodyIndex;
import fr.radi3nt.physics.core.state.DynamicsData;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

public class RelativeDistanceConstraint implements Constraint {

    private final RigidBodyIndex rigidBodyIndexA;
    private final RigidBodyIndex rigidBodyIndexB;
    private final Vector3f localSpaceAnchorA;
    private final Vector3f localSpaceAnchorB;
    private final float distance;

    private final DriftParameters driftParameters;
    private final CachingModuleProvider cachingProvider;

    public RelativeDistanceConstraint(RigidBodyIndex rigidBodyIndexA, RigidBodyIndex rigidBodyIndexB, Vector3f localSpaceAnchorA, Vector3f localSpaceAnchorB, float distance, DriftParameters driftParameters, CachingModuleProvider cachingProvider) {
        this.rigidBodyIndexA = rigidBodyIndexA;
        this.rigidBodyIndexB = rigidBodyIndexB;
        this.localSpaceAnchorA = localSpaceAnchorA;
        this.localSpaceAnchorB = localSpaceAnchorB;
        this.distance = distance;
        this.driftParameters = driftParameters;
        this.cachingProvider = cachingProvider;
    }

    @Override
    public ConstraintData[] compute(RigidBodyIsland island) {
        return new RelativeDistanceConstraintData[]{new RelativeDistanceConstraintData(rigidBodyIndexA.getIdentifiedData(island), rigidBodyIndexB.getIdentifiedData(island))};
    }

    @Override
    public RigidBodyIndex[] getConcernedBodies() {
        return new RigidBodyIndex[] {rigidBodyIndexA, rigidBodyIndexB};
    }

    private class RelativeDistanceConstraintData implements ConstraintData {

        private final IdentifiedDynamicsData indexA;
        private final IdentifiedDynamicsData indexB;
        private final Vector3f normal;
        private final Vector3f raCrossN;
        private final Vector3f rbCrossN;

        private final float correction;

        private RelativeDistanceConstraintData(IdentifiedDynamicsData indexA, IdentifiedDynamicsData indexB) {
            this.indexA = indexA;
            this.indexB = indexB;

            DynamicsData aData = indexA.data;
            DynamicsData bData = indexB.data;

            Vector3f aPosition = aData.getPosition();
            Vector3f bPosition = bData.getPosition();

            Quaternion aRotation = aData.getRotation();
            Quaternion bRotation = bData.getRotation();

            Vector3f ra = localSpaceAnchorA.duplicate();
            aRotation.transform(ra);
            Vector3f rb = localSpaceAnchorB.duplicate();
            bRotation.transform(rb);

            Vector3f pAB = aPosition.duplicate().add(ra).sub(bPosition).sub(rb);

            normal = pAB.duplicate().normalize();
            correction = distance - normal.dot(pAB);

            raCrossN = ra.duplicate().cross(normal);
            rbCrossN = rb.duplicate().cross(normal);

            System.out.println(correction);
        }

        @Override
        public StateConstraint[] getImpulses() {
            return new StateConstraint[] {
                    new StateConstraint(new IdentifiedDynamicsData[] {indexA, indexB}, new VectorNf[]{
                            new ArrayVectorNf(-normal.getX(), -normal.getY(), -normal.getZ(), -raCrossN.getX(), -raCrossN.getY(), -raCrossN.getZ()),
                            new ArrayVectorNf(normal.getX(), normal.getY(), normal.getZ(), rbCrossN.getX(), rbCrossN.getY(), rbCrossN.getZ()),
                    })
            };
        }

        @Override
        public DriftParameters[] getDriftParameters() {
            return new DriftParameters[] {driftParameters};
        }

        @Override
        public CachingModuleProvider getCachingConstraintModules() {
            return cachingProvider;
        }

        @Override
        public float[] getMax() {
            return new float[] {Float.MAX_VALUE};
        }

        @Override
        public float[] getMin() {
            return new float[] {-Float.MAX_VALUE};
        }

        @Override
        public float[] getCorrections() {
            return new float[] {correction};
        }
    }
}
