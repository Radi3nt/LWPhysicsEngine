package fr.radi3nt.physics.constraints.constraint.constraints;

import fr.radi3nt.maths.components.advanced.matrix.angle.Angle;
import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.constraints.constraint.ConstraintData;
import fr.radi3nt.physics.constraints.constraint.DriftParameters;
import fr.radi3nt.physics.constraints.constraint.caching.provider.CachingModuleProvider;
import fr.radi3nt.physics.constraints.constraint.index.IdentifiedDynamicsData;
import fr.radi3nt.physics.constraints.constraint.index.RigidBodyIndex;

import java.util.List;

public class LocalAxisSingleLimitConstraint extends ConeLimitConstraint {

    private final Vector3f localAxis;

    public LocalAxisSingleLimitConstraint(RigidBodyIndex indexA, RigidBodyIndex indexB, Quaternion rotationA, Quaternion rotationB, Angle angle, DriftParameters driftParameters, CachingModuleProvider provider, float limit, Vector3f localAxis) {
        super(indexA, indexB, rotationA, rotationB, angle, driftParameters, provider, limit);
        this.localAxis = localAxis;
    }


    protected void correctDiff(Quaternion currentDiff, List<ConstraintData> data, IdentifiedDynamicsData indexA, IdentifiedDynamicsData indexB) {

        Vector3f bAxis = localAxis.duplicate();
        Vector3f aAxis = localAxis.duplicate();
        indexB.data.getRotation().transform(bAxis);
        indexA.data.getRotation().transform(bAxis);


        Quaternion xQuat = currentDiff.getRotationComponentAboutAxis(bAxis);


        applyCorrection(data, indexA, indexB, (float) xQuat.getAngle().getRadiant(), aAxis);
    }
}
