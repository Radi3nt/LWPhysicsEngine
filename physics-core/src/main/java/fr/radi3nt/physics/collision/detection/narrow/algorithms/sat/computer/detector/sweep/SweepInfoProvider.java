package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.computer.detector.sweep;

import fr.radi3nt.maths.components.vectors.Vector3f;

public interface SweepInfoProvider {

    float getRelativeVelocityOnAxis(Vector3f axis);
    boolean noCollision(float enterT, float leaveT);

}
