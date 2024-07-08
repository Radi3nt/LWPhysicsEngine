package fr.radi3nt.physics.collision.detection.narrow.algorithms.gjk.shapes.provider;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.gjk.shapes.GjkProcessedShape;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.gjk.shapes.GjkVerticesProcessedShape;
import fr.radi3nt.physics.collision.detection.narrow.processed.ProcessedShapeProvider;
import fr.radi3nt.physics.collision.shape.shapes.BoxShape;
import fr.radi3nt.physics.collision.shape.shapes.CapsuleShape;
import fr.radi3nt.physics.collision.shape.shapes.CollisionShape;
import fr.radi3nt.physics.core.TransformedObject;

public class GjkCapsuleShapeProvider implements ProcessedShapeProvider<GjkProcessedShape> {

    public static final GjkCapsuleShapeProvider INSTANCE = new GjkCapsuleShapeProvider();

    private GjkCapsuleShapeProvider() {
    }

    @Override
    public boolean isSupported(CollisionShape shape) {
        return shape instanceof CapsuleShape;
    }

    @Override
    public GjkProcessedShape getShape(CollisionShape shape, TransformedObject transformedObject) {
        CapsuleShape capsuleShape = (CapsuleShape) shape;
        float height = capsuleShape.getHeight();

        Vector3f[] vertices = new Vector3f[] {
                new SimpleVector3f(0, -(height/2f), 0),
                new SimpleVector3f(0, height/2f, 0),
        };

        for (int i = 0; i < vertices.length; i++) {
            vertices[i] = transformedObject.toWorldSpace(vertices[i]);
        }

        return new GjkVerticesProcessedShape(vertices) {
            @Override
            public float transformDistance(float dist) {
                return super.transformDistance(dist)-capsuleShape.getRadius();
            }
        };
    }

}
