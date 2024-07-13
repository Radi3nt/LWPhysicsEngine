package fr.radi3nt.physics.collision.detection.narrow.algorithms.gjk.shapes.provider;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.gjk.shapes.GjkProcessedShape;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.gjk.shapes.GjkVerticesProcessedShape;
import fr.radi3nt.physics.collision.detection.narrow.processed.ProcessedShapeProvider;
import fr.radi3nt.physics.collision.shape.shapes.BoxShape;
import fr.radi3nt.physics.collision.shape.shapes.CollisionShape;
import fr.radi3nt.physics.collision.shape.shapes.TriangleShape;
import fr.radi3nt.physics.core.TransformedObject;

public class GjkTriangleShapeProvider implements ProcessedShapeProvider<GjkProcessedShape> {

    public static final GjkTriangleShapeProvider INSTANCE = new GjkTriangleShapeProvider();

    private GjkTriangleShapeProvider() {
    }

    @Override
    public boolean isSupported(CollisionShape shape) {
        return shape instanceof TriangleShape;
    }

    @Override
    public GjkProcessedShape getShape(CollisionShape shape, TransformedObject transformedObject) {
        TriangleShape triangleShape = (TriangleShape) shape;

        Vector3f[] vertices = new Vector3f[] {
            triangleShape.getVertex1(),
            triangleShape.getVertex2(),
            triangleShape.getVertex3()
        };
        for (int i = 0; i < vertices.length; i++) {
            vertices[i] = transformedObject.toWorldSpace(vertices[i]);
        }

        return new GjkVerticesProcessedShape(vertices);
    }

}
