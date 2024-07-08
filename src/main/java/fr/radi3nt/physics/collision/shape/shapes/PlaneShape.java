package fr.radi3nt.physics.collision.shape.shapes;

import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.vectors.Vector2f;
import fr.radi3nt.maths.components.vectors.Vector3f;

public class PlaneShape extends TransformedShape {

    private final Vector2f size;
    private final boolean oneWay;
    private final boolean edgeless;

    public PlaneShape(Vector3f offset, Quaternion rotation, Vector2f size, boolean oneWay, boolean edgeless) {
        super(offset, rotation);
        this.size = size;
        this.oneWay = oneWay;
        this.edgeless = edgeless;
    }

    public PlaneShape(Vector2f size, boolean oneWay, boolean edgeless) {
        this.size = size;
        this.oneWay = oneWay;
        this.edgeless = edgeless;
    }
}
