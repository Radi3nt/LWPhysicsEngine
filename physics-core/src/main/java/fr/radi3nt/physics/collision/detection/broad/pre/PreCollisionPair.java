package fr.radi3nt.physics.collision.detection.broad.pre;

import fr.radi3nt.physics.collision.shape.pre.PreCollisionData;
import fr.radi3nt.physics.core.TransformedObject;

public class PreCollisionPair {

    private final TransformedObject bodyA;
    private final TransformedObject bodyB;
    private PreCollisionData shapeA;
    private PreCollisionData shapeB;

    public PreCollisionPair(TransformedObject bodyA, TransformedObject bodyB) {
        this.bodyA = bodyA;
        this.bodyB = bodyB;
    }

    public static boolean isValid(PreCollisionHolder dataA, PreCollisionHolder dataB) {
        return PreCollisionPair.isValid(dataA.getPreCollisionShape(), dataB.getPreCollisionShape());
    }

    public static boolean isValid(PreCollisionShape shapeA, PreCollisionShape shapeB) {
        return shapeA!=null && shapeB!=null;
    }

    public PreCollisionPair from(PreCollisionHolder objA, PreCollisionHolder objB) {
        setShapeA(objA.getPreCollisionShape().toData(bodyA));
        setShapeB(objB.getPreCollisionShape().toData(bodyB));
        return this;
    }

    public PreCollisionPair from(PreCollisionData objA, PreCollisionData objB) {
        setShapeA(objA);
        setShapeB(objB);
        return this;
    }

    public void setShapeA(PreCollisionData shapeA) {
        this.shapeA = shapeA;
    }

    public void setShapeB(PreCollisionData shapeB) {
        this.shapeB = shapeB;
    }

    public TransformedObject getBodyA() {
        return bodyA;
    }

    public TransformedObject getBodyB() {
        return bodyB;
    }

    public PreCollisionData getShapeA() {
        return shapeA;
    }

    public PreCollisionData getShapeB() {
        return shapeB;
    }

    public void setShapeA(PreCollisionShape preCollisionShape) {
        setShapeA(preCollisionShape.toData(bodyA));
    }

    public void setShapeB(PreCollisionShape preCollisionShape) {
        setShapeB(preCollisionShape.toData(bodyB));
    }
}
