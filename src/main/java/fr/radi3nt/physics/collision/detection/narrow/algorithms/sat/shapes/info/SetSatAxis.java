package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.info;

import fr.radi3nt.maths.components.vectors.Vector3f;

public class SetSatAxis implements SatAxis {

    private final Vector3f axis;
    private final SatFace[] faces;

    public SetSatAxis(Vector3f axis, SatFace[] faces) {
        this.axis = axis;
        this.faces = faces;
    }

    @Override
    public Vector3f getAxis() {
        return axis;
    }

    @Override
    public SatFace[] getCorrespondingPlanes() {
        return faces;
    }
}
