package fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.relative;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.contact.ContactPair;
import fr.radi3nt.physics.collision.detection.narrow.sat.computer.CollisionGenerator;
import fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.CollisionDetector;
import fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.ManifoldPointBuilder;
import fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.normal.NormalSatCollisionDetector;
import fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.relative.RelativeSatCollisionDetector;
import fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.normal.NormalReuseManifoldPointBuilder;
import fr.radi3nt.physics.collision.shape.sat.shape.SatShapeObject;

public class CombiningCollisionGenerator implements CollisionGenerator {

    private final RelativeSatCollisionDetector relativeSatCollisionDetector = new RelativeSatCollisionDetector() {
        @Override
        public boolean testCollision(ContactPair pair, SatShapeObject shape1, SatShapeObject shape2) {
            boolean collision = super.testCollision(pair, shape1, shape2);
            if (collision)
                normalGen.computeVertices(pair, shape1, shape2);
            return collision;
        }
    };
    private final NormalReuseManifoldPointBuilder manifoldPointBuilder;
    private final NormalSatCollisionDetector normalGen;

    public CombiningCollisionGenerator(float penetrationEpsilon) {
        manifoldPointBuilder = new NormalReuseManifoldPointBuilder(normalGen = new NormalSatCollisionDetector() {
            @Override
            public float getCurrentOverlap() {
                return relativeSatCollisionDetector.getCurrentOverlap();
            }

            @Override
            public int getOverlapNormal() {
                return relativeSatCollisionDetector.getOverlapNormal();
            }

            @Override
            public Vector3f getMinimumTranslationAxis() {
                return relativeSatCollisionDetector.getMinimumTranslationAxis();
            }

            @Override
            public Vector3f[] getVerticesA() {
                return super.getVerticesA();
            }

            @Override
            public Vector3f[] getVerticesB() {
                return super.getVerticesB();
            }
        }, penetrationEpsilon);
    }


    @Override
    public CollisionDetector getDetector() {
        return relativeSatCollisionDetector;
    }

    @Override
    public ManifoldPointBuilder getPointBuilder() {
        return manifoldPointBuilder;
    }
}
