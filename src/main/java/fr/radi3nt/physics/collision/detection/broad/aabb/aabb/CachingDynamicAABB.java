package fr.radi3nt.physics.collision.detection.broad.aabb.aabb;

import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.mapping.AxisMapping;

import java.util.function.Consumer;

public class CachingDynamicAABB implements AABB {

    private final Consumer<AxisMapping> xMappingSupplier;
    private final Consumer<AxisMapping> yMappingSupplier;
    private final Consumer<AxisMapping> zMappingSupplier;
    private AxisMapping xMapping;
    private AxisMapping yMapping;
    private AxisMapping zMapping;

    public CachingDynamicAABB(Consumer<AxisMapping> xMappingSupplier, Consumer<AxisMapping> yMappingSupplier, Consumer<AxisMapping> zMappingSupplier) {
        this.xMappingSupplier = xMappingSupplier;
        this.yMappingSupplier = yMappingSupplier;
        this.zMappingSupplier = zMappingSupplier;
    }

    public AxisMapping getxMapping() {
        if (xMapping != null) {
            return xMapping;
        }

        return xMapping = getxMapping(new AxisMapping(0, 0));
    }

    @Override
    public AxisMapping getxMapping(AxisMapping mapping) {
        xMappingSupplier.accept(mapping);
        return mapping;
    }

    public AxisMapping getyMapping() {
        if (yMapping != null) {
            return yMapping;
        }

        return yMapping = getyMapping(new AxisMapping(0, 0));
    }

    @Override
    public AxisMapping getyMapping(AxisMapping mapping) {
        yMappingSupplier.accept(mapping);
        return mapping;
    }

    public AxisMapping getzMapping() {
        if (zMapping != null) {
            return zMapping;
        }

        return zMapping = getzMapping(new AxisMapping(0, 0));
    }

    @Override
    public AxisMapping getzMapping(AxisMapping mapping) {
        zMappingSupplier.accept(mapping);
        return mapping;
    }
}
