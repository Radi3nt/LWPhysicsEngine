package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.miscellaneous;

import fr.radi3nt.maths.components.vectors.Vector3f;

public class SatResult {

    private final boolean collision;
    private final Vector3f axis;
    private final double overlap;

    public SatResult(boolean collision, Vector3f axis, double overlap) {
        this.collision = collision;
        this.axis = axis;
        this.overlap = overlap;
    }

    public boolean isCollision() {
        return collision;
    }

    public Vector3f getAxis() {
        return axis;
    }

    public double getOverlap() {
        return overlap;
    }
}
