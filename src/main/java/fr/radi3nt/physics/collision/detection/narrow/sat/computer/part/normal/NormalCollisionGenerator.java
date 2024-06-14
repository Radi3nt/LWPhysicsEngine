package fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.normal;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.detection.narrow.sat.computer.CollisionGenerator;
import fr.radi3nt.physics.collision.detection.narrow.sat.computer.CollisionResult;
import fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.NoCollisionResult;
import fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.ShapedPair;
import fr.radi3nt.physics.collision.shape.sat.SatCollisionShape;
import fr.radi3nt.physics.collision.shape.sat.shape.SatShapeObject;

public class NormalCollisionGenerator implements CollisionGenerator {

    private final NormalSatCollisionDetector normalSatCollisionDetector = new NormalSatCollisionDetector();
    private final NormalReuseManifoldPointBuilder manifoldPointBuilder;

    public NormalCollisionGenerator() {
        manifoldPointBuilder = new NormalReuseManifoldPointBuilder();
    }

    @Override
    public CollisionResult test(GeneratedContactPair contactPair) {
        SatCollisionShape aShape = (SatCollisionShape) contactPair.shapeA;
        SatCollisionShape bShape = (SatCollisionShape) contactPair.shapeB;

        SatShapeObject sa = aShape.getShape();
        SatShapeObject sb = bShape.getShape();
        boolean collision = normalSatCollisionDetector.testCollision(contactPair, sa, sb);
        if (!collision)
            return NoCollisionResult.INSTANCE;
        NormalCollisionResult normalCollisionResult = new NormalCollisionResult(manifoldPointBuilder);
        normalCollisionResult.set(new ShapedPair(contactPair, sa, sb), normalSatCollisionDetector.getMinimumTranslationAxis(), normalSatCollisionDetector.getOverlapNormal(), normalSatCollisionDetector.getCurrentOverlap());
        return normalCollisionResult;
    }
}
