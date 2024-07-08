package fr.radi3nt.physics.constraints.constraint.constraints;

import fr.radi3nt.maths.components.advanced.matrix.Matrix3x3;
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

import static fr.radi3nt.maths.Maths.skew;

public class FixedPositionConstraint implements Constraint {

    private final RigidBodyIndex rigidBodyId;
    private final Vector3f worldSpaceAnchor;
    private final Vector3f localSpaceAnchor;
    private final DriftParameters driftParameters;
    private final CachingModuleProvider cachingProvider;

    private float max;
    private float min;

    public FixedPositionConstraint(RigidBodyIndex rigidBodyId, Vector3f worldSpaceAnchor, Vector3f localSpaceAnchor, DriftParameters driftParameters, CachingModuleProvider cachingProvider, float max, float min) {
        this.rigidBodyId = rigidBodyId;
        this.worldSpaceAnchor = worldSpaceAnchor;
        this.localSpaceAnchor = localSpaceAnchor;
        this.driftParameters = driftParameters;
        this.cachingProvider = cachingProvider;
        this.max = max;
        this.min = min;
    }

    public FixedPositionConstraint(RigidBodyIndex rigidBodyId, Vector3f worldSpaceAnchor, Vector3f localSpaceAnchor, DriftParameters driftParameters, CachingModuleProvider cachingProvider) {
        this(rigidBodyId, worldSpaceAnchor, localSpaceAnchor, driftParameters, cachingProvider, Float.MAX_VALUE, -Float.MAX_VALUE);
    }

    public void setMax(float max) {
        this.max = max;
    }

    public void setMin(float min) {
        this.min = min;
    }

    @Override
    public ConstraintData compute(RigidBodyIsland island) {
        IdentifiedDynamicsData identifiedData = rigidBodyId.getIdentifiedData(island);
        if (identifiedData==null)
            return null;
        return new FixedPositionConstraintData(identifiedData);
    }

    @Override
    public RigidBodyIndex[] getConcernedBodies() {
        return new RigidBodyIndex[] {rigidBodyId};
    }

    public class FixedPositionConstraintData implements ConstraintData {

        private final Vector3f angularVelocity;
        private final Vector3f derivedBodySpaceAnchor;
        private final Vector3f derivedWorldSpaceAnchor;
        private final IdentifiedDynamicsData index;

        public FixedPositionConstraintData(IdentifiedDynamicsData index) {
            this.index = index;

            DynamicsData rigidBodyData = index.data;
            Vector3f position = rigidBodyData.getPosition();
            Quaternion rotation = rigidBodyData.getRotation();

            this.angularVelocity = rigidBodyData.getAngularVelocity();

            this.derivedBodySpaceAnchor = localSpaceAnchor.duplicate();
            rotation.transform(derivedBodySpaceAnchor);

            this.derivedWorldSpaceAnchor = this.derivedBodySpaceAnchor.duplicate().add(position);
        }

        @Override
        public StateConstraint[] getImpulses() {
            Matrix3x3 skewedR = skew(derivedBodySpaceAnchor.duplicate().negate());

            return new StateConstraint[] {
                    new StateConstraint(new IdentifiedDynamicsData[] {index}, new VectorNf[]{
                            new ArrayVectorNf(1, 0, 0, 0, skewedR.get(1, 0), skewedR.get(2, 0))
                    }),
                    new StateConstraint(new IdentifiedDynamicsData[] {index}, new VectorNf[]{
                            new ArrayVectorNf(0, 1, 0, skewedR.get(0, 1), 0, skewedR.get(2, 1))
                    }),
                    new StateConstraint(new IdentifiedDynamicsData[] {index}, new VectorNf[]{
                            new ArrayVectorNf(0, 0, 1, skewedR.get(0, 2), skewedR.get(1, 2), 0)
                    })
            };
        }

        @Override
        public float[] getMax() {
            return new float[] {max, max, max};
        }

        @Override
        public float[] getMin() {
            return new float[] {min, min, min};
        }

        @Override
        public float[] getCorrections() {
            return new float[] {
                    derivedWorldSpaceAnchor.getX()-worldSpaceAnchor.getX(),
                    derivedWorldSpaceAnchor.getY()-worldSpaceAnchor.getY(),
                    derivedWorldSpaceAnchor.getZ()-worldSpaceAnchor.getZ()
            };
        }

        @Override
        public StateConstraint[] getForces() {
            Vector3f pointRotationalVelocity = angularVelocity.duplicate().cross(derivedBodySpaceAnchor.duplicate());
            Matrix3x3 skewedROmega = skew(pointRotationalVelocity.duplicate().negate());
            return new StateConstraint[] {
                    new StateConstraint(new IdentifiedDynamicsData[] {index}, new VectorNf[]{
                            new ArrayVectorNf(0, 0, 0, 0, skewedROmega.get(1, 0), skewedROmega.get(2, 0))
                    }),
                    new StateConstraint(new IdentifiedDynamicsData[] {index}, new VectorNf[]{
                            new ArrayVectorNf(0, 0, 0, skewedROmega.get(0, 1), 0, skewedROmega.get(2, 1))
                    }),
                    new StateConstraint(new IdentifiedDynamicsData[] {index}, new VectorNf[]{
                            new ArrayVectorNf(0, 0, 0, skewedROmega.get(0, 2), skewedROmega.get(1, 2), 0)
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
