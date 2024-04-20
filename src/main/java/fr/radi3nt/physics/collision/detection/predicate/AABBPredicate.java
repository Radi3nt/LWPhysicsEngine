package fr.radi3nt.physics.collision.detection.predicate;

import fr.radi3nt.maths.aabb.AABB;
import fr.radi3nt.maths.aabb.AxisMapping;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.function.Predicate;

public class AABBPredicate implements Predicate<RigidBody> {

    private final AABB aabb;

    public AABBPredicate(AABB aabb) {
        this.aabb = aabb;
    }

    @Override
    public boolean test(RigidBody rigidBody) {

        Vector3f position = rigidBody.getPosition();

        AxisMapping xMapping = aabb.getxMapping();
        AxisMapping yMapping = aabb.getyMapping();
        AxisMapping zMapping = aabb.getzMapping();

        if (!xMapping.inside(position.getX()))
            return false;
        if (!yMapping.inside(position.getY()))
            return false;
        if (!zMapping.inside(position.getZ()))
            return false;

        return true;
    }
}
