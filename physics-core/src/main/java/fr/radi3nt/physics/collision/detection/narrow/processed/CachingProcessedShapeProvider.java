package fr.radi3nt.physics.collision.detection.narrow.processed;

import fr.radi3nt.physics.collision.shape.shapes.CollisionShape;
import fr.radi3nt.physics.core.TransformedObject;

import java.util.Collection;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.ConcurrentHashMap;

public class CachingProcessedShapeProvider<T extends ProcessedShape> implements ProcessedShapeProvider<T> {

    private final Map<TransformedPair, T> cachedShapes = new ConcurrentHashMap<>();
    private final ProcessedShapeProvider<T> provider;

    public CachingProcessedShapeProvider(ProcessedShapeProvider<T> provider) {
        this.provider = provider;
    }

    public static <T extends ProcessedShape> CachingProcessedShapeProvider<T> from(ProcessedShapeProvider<T> provider, Collection<CachingProcessedShapeProvider<?>> providers) {
        CachingProcessedShapeProvider<T> caching = new CachingProcessedShapeProvider<>(provider);
        providers.add(caching);
        return caching;
    }

    @Override
    public boolean isSupported(CollisionShape shape) {
        return provider.isSupported(shape);
    }

    @Override
    public T getShape(CollisionShape shape, TransformedObject transformedObject) {
        return cachedShapes.computeIfAbsent(new TransformedPair(transformedObject, shape), currentTransform -> provider.getShape(shape, currentTransform.transformedObject));
    }

    public void clearCache() {
        cachedShapes.clear();
    }

    public static class TransformedPair {

        private final TransformedObject transformedObject;
        private final CollisionShape shape;

        public TransformedPair(TransformedObject transformedObject, CollisionShape shape) {
            this.transformedObject = transformedObject;
            this.shape = shape;
        }

        @Override
        public final boolean equals(Object o) {
            if (this == o) return true;
            if (!(o instanceof TransformedPair)) return false;

            TransformedPair that = (TransformedPair) o;
            return Objects.equals(transformedObject, that.transformedObject) && Objects.equals(shape, that.shape);
        }

        @Override
        public int hashCode() {
            int result = Objects.hashCode(transformedObject);
            result = 31 * result + Objects.hashCode(shape);
            return result;
        }
    }
}
