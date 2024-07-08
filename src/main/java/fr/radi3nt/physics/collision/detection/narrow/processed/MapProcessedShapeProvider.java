package fr.radi3nt.physics.collision.detection.narrow.processed;

import fr.radi3nt.physics.collision.shape.shapes.CollisionShape;
import fr.radi3nt.physics.core.TransformedObject;

import java.util.Map;

public class MapProcessedShapeProvider<T extends ProcessedShape> implements ProcessedShapeProvider<T> {

    private final Map<Class<? extends CollisionShape>, ProcessedShapeProvider<? extends T>> shapeProviderMap;

    public MapProcessedShapeProvider(Map<Class<? extends CollisionShape>, ProcessedShapeProvider<? extends T>> shapeProviderMap) {
        this.shapeProviderMap = shapeProviderMap;
    }

    @Override
    public boolean isSupported(CollisionShape shape) {
        return shapeProviderMap.containsKey(shape.getClass());
    }

    @Override
    public T getShape(CollisionShape shape, TransformedObject transformedObject) {
        return shapeProviderMap.get(shape.getClass()).getShape(shape, transformedObject);
    }
}
