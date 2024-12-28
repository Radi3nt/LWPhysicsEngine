package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.computer.detector.sweep;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;

public class RaySweepInfoProvider implements SweepInfoProvider {
    
    private final Vector3f ray = new SimpleVector3f();

    public RaySweepInfoProvider() {
    }

    public RaySweepInfoProvider(Vector3f ray) {
        this.ray.copy(ray);
    }

    @Override
    public float getRelativeVelocityOnAxis(Vector3f axis) {
        return axis.dot(ray);
    }

    @Override
    public boolean noCollision(float enterT, float leaveT) {
        return leaveT<0 || enterT>leaveT;
    }

    public Vector3f getRay() {
        return ray;
    }
}
