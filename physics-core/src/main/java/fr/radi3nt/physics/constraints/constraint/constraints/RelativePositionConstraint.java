package fr.radi3nt.physics.constraints.constraint.constraints;

import fr.radi3nt.maths.components.advanced.matrix.Matrix3x3;
import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.arbitrary.VectorNf;
import fr.radi3nt.maths.components.arbitrary.vector.ArrayVectorNf;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.constraints.constraint.*;
import fr.radi3nt.physics.constraints.constraint.caching.provider.CachingModuleProvider;
import fr.radi3nt.physics.constraints.constraint.index.IdentifiedDynamicsData;
import fr.radi3nt.physics.constraints.constraint.index.RigidBodyIndex;
import fr.radi3nt.physics.core.state.DynamicsData;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

import static fr.radi3nt.maths.Maths.skew;

public class RelativePositionConstraint implements Constraint {

    private final RigidBodyIndex rigidBodyIdA;
    private final RigidBodyIndex rigidBodyIdB;
    private final Vector3f localSpaceAnchorA;
    private final Vector3f localSpaceAnchorB;
    private final DriftParameters driftParameters;
    private final CachingModuleProvider cachingProvider;
    private final float limit;

    public RelativePositionConstraint(RigidBodyIndex rigidBodyIdA, RigidBodyIndex rigidBodyIdB, Vector3f localSpaceAnchorA, Vector3f localSpaceAnchorB, DriftParameters driftParameters, CachingModuleProvider cachingProvider, float limit) {
        this.rigidBodyIdA = rigidBodyIdA;
        this.rigidBodyIdB = rigidBodyIdB;
        this.localSpaceAnchorA = localSpaceAnchorA;
        this.localSpaceAnchorB = localSpaceAnchorB;
        this.driftParameters = driftParameters;
        this.cachingProvider = cachingProvider;
        this.limit = limit;
    }


    @Override
    public ConstraintData[] compute(RigidBodyIsland island) {
        return new RelativePositionConstraintData[]{new RelativePositionConstraintData(rigidBodyIdA.getIdentifiedData(island), rigidBodyIdB.getIdentifiedData(island))};
    }

    @Override
    public RigidBodyIndex[] getConcernedBodies() {
        return new RigidBodyIndex[] {rigidBodyIdA, rigidBodyIdB};
    }

    public class RelativePositionConstraintData implements ForceConstraintData {

        private final Vector3f angularVelocityA;
        private final Vector3f angularVelocityB;

        private final Vector3f derivedBodySpaceAnchorA;
        private final Vector3f derivedBodySpaceAnchorB;

        private final Vector3f derivedWorldSpaceAnchorA;
        private final Vector3f derivedWorldSpaceAnchorB;
        private final IdentifiedDynamicsData indexA;
        private final IdentifiedDynamicsData indexB;

        public RelativePositionConstraintData(IdentifiedDynamicsData indexA, IdentifiedDynamicsData indexB) {
            this.indexA = indexA;
            this.indexB = indexB;

            DynamicsData aData = indexA.data;
            DynamicsData bData = indexB.data;

            Vector3f aPosition = aData.getPosition();
            Vector3f bPosition = bData.getPosition();

            Quaternion aRotation = aData.getRotation();
            Quaternion bRotation = bData.getRotation();

            this.angularVelocityA = aData.getAngularVelocity();
            this.derivedBodySpaceAnchorA = localSpaceAnchorA.duplicate();
            aRotation.transform(derivedBodySpaceAnchorA);

            this.angularVelocityB = bData.getAngularVelocity();
            this.derivedBodySpaceAnchorB = localSpaceAnchorB.duplicate();
            bRotation.transform(derivedBodySpaceAnchorB);

            this.derivedWorldSpaceAnchorA = this.derivedBodySpaceAnchorA.duplicate().add(aPosition);
            this.derivedWorldSpaceAnchorB = this.derivedBodySpaceAnchorB.duplicate().add(bPosition);
        }

        @Override
        public StateConstraint[] getImpulses() {
            Matrix3x3 skewedRA = skew(derivedBodySpaceAnchorA.duplicate());
            Matrix3x3 skewedRB = skew(derivedBodySpaceAnchorB.duplicate().negate());

            return new StateConstraint[] {
                    new StateConstraint(new IdentifiedDynamicsData[] {indexA, indexB}, new VectorNf[]{
                            new ArrayVectorNf(-1, 0, 0, 0, skewedRA.get(1, 0), skewedRA.get(2, 0)),
                            new ArrayVectorNf(1, 0, 0, 0, skewedRB.get(1, 0), skewedRB.get(2, 0))
                    }),
                    new StateConstraint(new IdentifiedDynamicsData[] {indexA, indexB}, new VectorNf[]{
                            new ArrayVectorNf(0, -1, 0, skewedRA.get(0, 1), 0, skewedRA.get(2, 1)),
                            new ArrayVectorNf(0, 1, 0, skewedRB.get(0, 1), 0, skewedRB.get(2, 1))
                    }),
                    new StateConstraint(new IdentifiedDynamicsData[] {indexA, indexB}, new VectorNf[]{
                            new ArrayVectorNf(0, 0, -1, skewedRA.get(0, 2), skewedRA.get(1, 2), 0),
                            new ArrayVectorNf(0, 0, 1, skewedRB.get(0, 2), skewedRB.get(1, 2), 0)
                    })
            };
        }

        @Override
        public float[] getMax() {
            return new float[] {limit, limit, limit};
        }

        @Override
        public float[] getMin() {
            return new float[] {-limit, -limit, -limit};
        }

        @Override
        public float[] getCorrections() {
            return new float[] {
                    derivedWorldSpaceAnchorB.getX() - derivedWorldSpaceAnchorA.getX(),
                    derivedWorldSpaceAnchorB.getY() - derivedWorldSpaceAnchorA.getY(),
                    derivedWorldSpaceAnchorB.getZ() - derivedWorldSpaceAnchorA.getZ()
            };
        }

        @Override
        public StateConstraint[] getForces() {
            Vector3f pointRotationalVelocityA = angularVelocityA.duplicate().cross(derivedBodySpaceAnchorA.duplicate());
            Vector3f pointRotationalVelocityB = angularVelocityB.duplicate().cross(derivedBodySpaceAnchorB.duplicate());
            Matrix3x3 skewedROmegaA = skew(pointRotationalVelocityA.duplicate());
            Matrix3x3 skewedROmegaB = skew(pointRotationalVelocityB.duplicate().negate());
            return new StateConstraint[] {
                    new StateConstraint(new IdentifiedDynamicsData[] {indexA, indexB}, new VectorNf[]{
                            new ArrayVectorNf(0, 0, 0, 0, skewedROmegaA.get(1, 0), skewedROmegaA.get(2, 0)),
                            new ArrayVectorNf(0, 0, 0, 0, skewedROmegaB.get(1, 0), skewedROmegaB.get(2, 0))
                    }),
                    new StateConstraint(new IdentifiedDynamicsData[] {indexA, indexB}, new VectorNf[]{
                            new ArrayVectorNf(0, 0, 0, skewedROmegaA.get(0, 1), 0, skewedROmegaA.get(2, 1)),
                            new ArrayVectorNf(0, 0, 0, skewedROmegaB.get(0, 1), 0, skewedROmegaB.get(2, 1))
                    }),
                    new StateConstraint(new IdentifiedDynamicsData[] {indexA, indexB}, new VectorNf[]{
                            new ArrayVectorNf(0, 0, 0, skewedROmegaA.get(0, 2), skewedROmegaA.get(1, 2), 0),
                            new ArrayVectorNf(0, 0, 0, skewedROmegaB.get(0, 2), skewedROmegaB.get(1, 2), 0)
                    })
            };
        }

        @Override
        public DriftParameters[] getDriftParameters() {
            return new DriftParameters[] {
                    driftParameters,
                    driftParameters,
                    driftParameters
            };
        }

        @Override
        public CachingModuleProvider getCachingConstraintModules() {
            return cachingProvider;
        }
    }
}
