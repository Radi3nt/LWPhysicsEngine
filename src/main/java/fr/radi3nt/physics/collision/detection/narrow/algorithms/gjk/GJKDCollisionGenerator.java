package fr.radi3nt.physics.collision.detection.narrow.algorithms.gjk;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.gjk.shapes.GjkProcessedShape;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class GJKDCollisionGenerator {

    private static final Vector3f START_AXIS = new SimpleVector3f(0, -1, 0);
    private static final int MAX_ITERATIONS = 32;

    public CollisionResult test(GjkProcessedShape shape1, GjkProcessedShape shape2) {
        GjkPoint supportPoint = getSupportPoint(shape1, shape2, START_AXIS);

        GJKSimplex simplex = new GJKSimplex();
        simplex.addFront(supportPoint);

        GjkPoint lastPoint = null;

        Vector3f currentDirection = supportPoint.getPoint().duplicate().negate();

        int iterations = 0;
        while (true) {
            supportPoint = getSupportPoint(shape1, shape2, currentDirection);
            Vector3f vMinusP = supportPoint.getPoint().duplicate().sub(currentDirection.duplicate().negate());

            if (vMinusP.dot(currentDirection.duplicate())<=0 || supportPoint.equals(lastPoint) || simplex.alreadyHasPoint(supportPoint) || iterations>MAX_ITERATIONS) {
                float distanceToOrigin = simplexDistanceToOrigin(simplex, shape1, shape2);
                return new CollisionResult(distanceToOrigin, simplex, false);
            }
            simplex.addFront(supportPoint);
            GJKSimplex duplicate = simplex.duplicate();
            boolean containsOrigin = nearestSimplex(simplex, currentDirection);
            if (containsOrigin) {
                return new CollisionResult(Float.POSITIVE_INFINITY, simplex, true);
            }

            lastPoint = null;
            for (GjkPoint point : duplicate.getPoints()) {
                if (!simplex.alreadyHasPoint(point)) {
                    lastPoint = point;
                    break;
                }
            }

            iterations++;
        }
    }

    private float simplexDistanceToOrigin(GJKSimplex simplex, GjkProcessedShape shape1, GjkProcessedShape shape2) {
        float dist = simplex.distanceToOrigin();
        dist = shape1.transformDistance(dist);
        dist = shape2.transformDistance(dist);
        return dist;
    }

    private boolean nearestSimplex(GJKSimplex simplex, Vector3f currentDirection) {
        return simplex.nearestSimplex(currentDirection);
    }

    private GjkPoint getSupportPoint(GjkProcessedShape shapeA, GjkProcessedShape shapeB, Vector3f direction) {
        List<GjkProcessedShape.FurthestPoint> furthestA = shapeA.furthestPointAlongAxis(direction);
        List<GjkProcessedShape.FurthestPoint> furthestB = shapeB.furthestPointAlongAxis(direction.duplicate().negate());

        return GjkPoint.from(furthestA, furthestB, direction);
    }

    public static class CollisionResult {

        public float closestDistance;
        public GJKSimplex collidingSimplex;
        public boolean collided;

        public CollisionResult(float closestDistance, GJKSimplex collidingSimplex, boolean collided) {
            this.closestDistance = closestDistance;
            this.collidingSimplex = collidingSimplex;
            this.collided = collided;
        }
    }

}
