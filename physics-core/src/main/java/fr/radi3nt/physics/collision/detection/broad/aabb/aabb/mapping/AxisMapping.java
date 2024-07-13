package fr.radi3nt.physics.collision.detection.broad.aabb.aabb.mapping;

public class AxisMapping implements ProjectionSettable {

    private float min;
    private float max;

    public AxisMapping(float min, float max) {
        this.min = min;
        this.max = max;
    }

    public void extend(float added) {
        min-=added;
        max+=added;
    }

    public boolean inside(float pos) {
        return this.min <= pos && this.max>=pos;
    }

    public boolean intersect(AxisMapping axisMapping) {
        return (min <= axisMapping.max && max >= axisMapping.min);
    }

    public boolean intersect(AxisMapping axisMapping, float offset) {
        return (min+offset <= axisMapping.max && max+offset >= axisMapping.min);
    }

    public float getMin() {
        return min;
    }

    public float getMax() {
        return max;
    }

    public void set(float min, float max) {
        this.min = min;
        this.max = max;
    }

    @Override
    public void setMin(float min) {
        this.min = min;
    }

    @Override
    public void setMax(float max) {
        this.max = max;
    }

    @Override
    public String toString() {
        return "AxisMapping{" +
                "min=" + min +
                ", max=" + max +
                '}';
    }
}
