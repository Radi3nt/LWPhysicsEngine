package fr.radi3nt.physics.constraints.constraint.constraints;

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

public class NoPenetrationConstraint implements Constraint {

    private final DriftParameters driftParameters;
    private final Vector3f normal;

    private final Vector3f ra;
    private final Vector3f rb;

    private final RigidBodyIndex attachedA;
    private final RigidBodyIndex attachedB;

    private final CachingModuleProvider cachingProvider;

    public NoPenetrationConstraint(DriftParameters driftParameters, Vector3f normal, Vector3f ra, Vector3f rb, RigidBodyIndex attachedA, RigidBodyIndex attachedB, CachingModuleProvider cachingProvider) {
        this.driftParameters = driftParameters;
        this.normal = normal;
        this.ra = ra;
        this.rb = rb;
        this.attachedA = attachedA;
        this.attachedB = attachedB;
        this.cachingProvider = cachingProvider;
    }

    @Override
    public ConstraintData[] compute(RigidBodyIsland island) {
        return new NoPenetrationConstraintData[]{new NoPenetrationConstraintData(attachedA.getIdentifiedData(island), attachedB.getIdentifiedData(island))};
    }

    @Override
    public RigidBodyIndex[] getConcernedBodies() {
        return new RigidBodyIndex[] {attachedA, attachedB};
    }

    public class NoPenetrationConstraintData implements ConstraintData {

        private final Vector3f bodyLocationA;
        private final Vector3f bodyLocationB;

        private final boolean shouldIncludeA;
        private final boolean shouldIncludeB;

        private final IdentifiedDynamicsData attachedAId;
        private final IdentifiedDynamicsData attachedBId;

        public NoPenetrationConstraintData(IdentifiedDynamicsData attachedAId, IdentifiedDynamicsData attachedBId) {
            DynamicsData a = attachedAId.data;
            DynamicsData b = attachedBId.data;

            bodyLocationA = a.getPosition();
            bodyLocationB = b.getPosition();

            shouldIncludeA = a.getBodyProperties().inverseMass!=0 && !attachedAId.sleepingData.isSleeping();
            shouldIncludeB = b.getBodyProperties().inverseMass!=0 && !attachedBId.sleepingData.isSleeping();
            this.attachedAId = attachedAId;
            this.attachedBId = attachedBId;
        }

        @Override
        public float[] getMax() {
            return new float[] {Float.MAX_VALUE};
        }

        @Override
        public float[] getMin() {
            return new float[] {0};
        }

        @Override
        public float[] getCorrections() {
            return new float[] {normal.dot(bodyLocationB.duplicate().add(rb).sub(bodyLocationA.duplicate().add(ra)))};
        }

        @Override
        public StateConstraint[] getImpulses() {
            int count = shouldIncludeA && shouldIncludeB ? 2 : 1;
            VectorNf[] states = new VectorNf[count];
            IdentifiedDynamicsData[] ids = new IdentifiedDynamicsData[count];
            int index = 0;
            if (shouldIncludeA) {
                Vector3f raCrossN = ra.duplicate().cross(normal);
                states[index] = new ArrayVectorNf(-normal.getX(), -normal.getY(), -normal.getZ(), -raCrossN.getX(), -raCrossN.getY(), -raCrossN.getZ());
                ids[index] = attachedAId;
                index++;
            }
            if (shouldIncludeB) {
                Vector3f rbCrossN = rb.duplicate().cross(normal);
                states[index] = new ArrayVectorNf(normal.getX(), normal.getY(), normal.getZ(), rbCrossN.getX(), rbCrossN.getY(), rbCrossN.getZ());
                ids[index] = attachedBId;
            }
            return new StateConstraint[] {
                    new StateConstraint(ids, states)
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

    }
}
