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

public class FrictionConstraint implements Constraint {

    private final Vector3f tangent1;
    private final Vector3f tangent2;
    private final float relativeVelocity;
    private final float frictionCoefficient;

    private final Vector3f ra;
    private final Vector3f rb;

    private final RigidBodyIndex attachedA;
    private final RigidBodyIndex attachedB;

    private final CachingModuleProvider cachingProvider;

    public FrictionConstraint(Vector3f tangent1, Vector3f tangent2, float relativeVelocity, float frictionCoefficient, Vector3f ra, Vector3f rb, RigidBodyIndex attachedA, RigidBodyIndex attachedB, CachingModuleProvider cachingProvider) {
        this.tangent1 = tangent1;
        this.tangent2 = tangent2;
        this.relativeVelocity = relativeVelocity;
        this.frictionCoefficient = frictionCoefficient;
        this.ra = ra;
        this.rb = rb;
        this.attachedA = attachedA;
        this.attachedB = attachedB;
        this.cachingProvider = cachingProvider;
    }

    @Override
    public ConstraintData compute(RigidBodyIsland island) {
        return new FrictionConstraintData(attachedA.getIdentifiedData(island), attachedB.getIdentifiedData(island));
    }

    @Override
    public RigidBodyIndex[] getConcernedBodies() {
        return new RigidBodyIndex[] {attachedA, attachedB};
    }

    public class FrictionConstraintData implements ConstraintData {

        private final Vector3f bodyLocationA;
        private final Vector3f bodyLocationB;

        private final boolean shouldIncludeA;
        private final boolean shouldIncludeB;

        private final float maxBoundT1;
        private final float maxBoundT2;

        private final IdentifiedDynamicsData attachedAId;
        private final IdentifiedDynamicsData attachedBId;

        public FrictionConstraintData(IdentifiedDynamicsData attachedAId, IdentifiedDynamicsData attachedBId) {
            DynamicsData a = attachedAId.data;
            DynamicsData b = attachedBId.data;

            bodyLocationA = a.getPosition();
            bodyLocationB = b.getPosition();

            shouldIncludeA = a.getBodyProperties().inverseMass!=0;
            shouldIncludeB = b.getBodyProperties().inverseMass!=0;

            this.attachedAId = attachedAId;
            this.attachedBId = attachedBId;
            this.maxBoundT1 = relativeVelocity;
            this.maxBoundT2 = relativeVelocity;
        }

        @Override
        public float[] getMax() {
            return new float[] {frictionCoefficient* maxBoundT1, frictionCoefficient* maxBoundT2};
        }

        @Override
        public float[] getMin() {
            return new float[] {-frictionCoefficient* maxBoundT1, -frictionCoefficient* maxBoundT2};
        }

        @Override
        public float[] getCorrections() {
            return new float[] {0, 0};
        }

        @Override
        public StateConstraint[] getImpulses() {
            int count = shouldIncludeA && shouldIncludeB ? 2 : 1;
            VectorNf[] states1 = new VectorNf[count];
            VectorNf[] states2 = new VectorNf[count];
            IdentifiedDynamicsData[] ids = new IdentifiedDynamicsData[count];
            int index = 0;
            if (shouldIncludeA) {
                Vector3f raCrossN1 = ra.duplicate().cross(tangent1);
                Vector3f raCrossN2 = ra.duplicate().cross(tangent2);
                states1[index] = new ArrayVectorNf(-tangent1.getX(), -tangent1.getY(), -tangent1.getZ(), -raCrossN1.getX(), -raCrossN1.getY(), -raCrossN1.getZ());
                states2[index] = new ArrayVectorNf(-tangent2.getX(), -tangent2.getY(), -tangent2.getZ(), -raCrossN2.getX(), -raCrossN2.getY(), -raCrossN2.getZ());
                ids[index] = attachedAId;
                index++;
            }
            if (shouldIncludeB) {
                Vector3f rbCrossN1 = rb.duplicate().cross(tangent1);
                Vector3f rbCrossN2 = rb.duplicate().cross(tangent2);
                states1[index] = new ArrayVectorNf(tangent1.getX(), tangent1.getY(), tangent1.getZ(), rbCrossN1.getX(), rbCrossN1.getY(), rbCrossN1.getZ());
                states2[index] = new ArrayVectorNf(tangent2.getX(), tangent2.getY(), tangent2.getZ(), rbCrossN2.getX(), rbCrossN2.getY(), rbCrossN2.getZ());
                ids[index] = attachedBId;
            }
            return new StateConstraint[] {
                    new StateConstraint(ids, states1),
                    new StateConstraint(ids, states2)
            };
        }

        @Override
        public StateConstraint[] getForces() {
            return new StateConstraint[0];
        }

        @Override
        public DriftParameters[] getDriftParameters() {
            return new DriftParameters[] {new DriftParameters(0f), new DriftParameters(0f)};
        }

        @Override
        public CachingModuleProvider getCachingConstraintModules() {
            return cachingProvider;
        }
    }
}
