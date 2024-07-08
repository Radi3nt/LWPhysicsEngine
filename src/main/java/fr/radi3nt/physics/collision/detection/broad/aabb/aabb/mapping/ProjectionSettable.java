package fr.radi3nt.physics.collision.detection.broad.aabb.aabb.mapping;

public interface ProjectionSettable {

    void set(float min, float max);
    void setMin(float min);
    void setMax(float max);

}
