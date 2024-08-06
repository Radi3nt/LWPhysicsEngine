package fr.radi3nt.physics.collision.detection.narrow.algorithms.gjk.shapes.provider;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.gjk.shapes.GjkProcessedShape;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.gjk.shapes.GjkVerticesProcessedShape;
import fr.radi3nt.physics.collision.detection.narrow.processed.ProcessedShapeProvider;
import fr.radi3nt.physics.collision.shape.shapes.BoxShape;
import fr.radi3nt.physics.collision.shape.shapes.CollisionShape;
import fr.radi3nt.physics.core.TransformedObject;

public class GjkBoxShapeProvider implements ProcessedShapeProvider<GjkProcessedShape> {

    public static final GjkBoxShapeProvider INSTANCE = new GjkBoxShapeProvider();

    private GjkBoxShapeProvider() {
    }

    @Override
    public boolean isSupported(CollisionShape shape) {
        return shape instanceof BoxShape;
    }

    @Override
    public GjkProcessedShape getShape(CollisionShape shape, TransformedObject transformedObject) {
        BoxShape boxShape = (BoxShape) shape;
        Vector3f size = boxShape.getSize();

        Vector3f[] vertices = new Vector3f[8];
        for (int i = 0; i < vertices.length; i++) {
            Vector3f localSpacePos = new SimpleVector3f((i % 2) * 2 - 1, ((i / 2) % 2) * 2 - 1, ((i / 2 / 2) % 2) * 2 - 1).mul(size);
            boxShape.getRotation().transform(localSpacePos);
            vertices[i] = transformedObject.toWorldSpace(localSpacePos.add(boxShape.getOffset()));
        }

        return new GjkVerticesProcessedShape(vertices);
    }

}
