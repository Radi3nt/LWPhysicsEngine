package fr.radi3nt.physics.collision.detection.broad.pre;

import fr.radi3nt.physics.core.TransformedObject;

public class PreCollisionPair {

    private final TransformedObject bodyA;
    private final TransformedObject bodyB;
    private PreCollisionShape shapeA;
    private PreCollisionShape shapeB;

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
        setShapeA(objA.getPreCollisionShape());
        setShapeB(objB.getPreCollisionShape());
        return this;
    }

    public void setShapeA(PreCollisionShape shapeA) {
        this.shapeA = shapeA;
    }

    public void setShapeB(PreCollisionShape shapeB) {
        this.shapeB = shapeB;
    }

    public TransformedObject getBodyA() {
        return bodyA;
    }

    public TransformedObject getBodyB() {
        return bodyB;
    }

    public PreCollisionShape getShapeA() {
        return shapeA;
    }

    public PreCollisionShape getShapeB() {
        return shapeB;
    }
}
