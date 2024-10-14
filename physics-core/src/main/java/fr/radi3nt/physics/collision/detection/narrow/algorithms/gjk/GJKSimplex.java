package fr.radi3nt.physics.collision.detection.narrow.algorithms.gjk;

import fr.radi3nt.maths.Maths;
import fr.radi3nt.maths.components.vectors.Vector3f;

public class GJKSimplex {

    private static final int POINT_DIMENSION = 0;
    private static final int LINE_DIMENSION = 1;
    private static final int TRIANGLE_DIMENSION = 2;
    private static final int TETRAHEDRON_DIMENSION = 3;
    private final PointBag points;

    public GJKSimplex() {
        points = new PointBag();
    }

    public GJKSimplex(PointBag points) {
        this.points = points;
    }

    public boolean nearestSimplex(Vector3f direction) {
        if (isLine())
            return line(direction);
        if (isTriangle())
            return triangle(direction);
        if (isTetrahedron())
            return tetrahedron(direction);
        throw new UnsupportedOperationException("Simplex is neither a line, a triangle, or a tetrahedron");
    }

    public float distanceToOrigin() {
        if (isLine())
            return distLine();
        if (isTriangle())
            return distTriangle();
        if (isTetrahedron())
            return distTetrahedron();
        return points.get(0).getPoint().length();
    }

    private float distTetrahedron() {
        Vector3f a = points.get(0).getPoint();
        Vector3f b = points.get(1).getPoint();
        Vector3f c = points.get(2).getPoint();
        Vector3f d = points.get(3).getPoint();

        float dabc = distTriangle(a, b, c);
        float dbcd = distTriangle(b, c, d);
        float dcda = distTriangle(c, d, a);
        float ddab = distTriangle(d, a, b);

        return Math.min(Math.min(Math.min(dabc, dbcd), dcda), ddab);
    }

    private float distTriangle() {
        Vector3f a = points.get(0).getPoint();
        Vector3f b = points.get(1).getPoint();
        Vector3f c = points.get(2).getPoint();

        return distTriangle(a, b, c);
    }

    private static float distTriangle(Vector3f a, Vector3f b, Vector3f c) {
        float dab = distLine(a, b);
        float dbc = distLine(b, c);
        float dca = distLine(c, a);

        Vector3f ab = b.duplicate().sub(a);
        Vector3f ac = c.duplicate().sub(a);
        Vector3f ao = a.duplicate().negate();

        Vector3f abc = ab.duplicate().cross(ac);
        abc.normalizeSafely();
        float proj = Float.MAX_VALUE;
        if (abc.lengthSquared()!=0)
            proj = distProjected(abc, ao);

        return Math.min(Math.min(Math.min(dab, dbc), dca), proj);
    }

    private float distLine() {
        Vector3f a = points.get(0).getPoint();
        Vector3f b = points.get(1).getPoint();

        return distLine(a, b);
    }

    public static float distLine(Vector3f a, Vector3f b) {

        Vector3f ab = b.duplicate().sub(a);
        Vector3f ao = a.duplicate().negate();

        float length = ab.lengthSquared();

        float projectedPoint = ab.dot(ao)/length;
        projectedPoint = Maths.clamp(projectedPoint, 0, 1);
        Vector3f closestPoint = ab.duplicate().mul(projectedPoint).add(a);

        float distance = closestPoint.length();
        return distance;
    }

    private static float distProjected(Vector3f ab, Vector3f ao) {
        return ab.dot(ao);
    }

    private boolean tetrahedron(Vector3f direction) {
        GjkPoint aP = points.get(0);
        GjkPoint bP = points.get(1);
        GjkPoint cP = points.get(2);
        GjkPoint dP = points.get(3);

        Vector3f a = aP.getPoint();
        Vector3f b = bP.getPoint();
        Vector3f c = cP.getPoint();
        Vector3f d = dP.getPoint();

        Vector3f ab = b.duplicate().sub(a);
        Vector3f ac = c.duplicate().sub(a);
        Vector3f ad = d.duplicate().sub(a);
        Vector3f ao = a.duplicate().negate();

        Vector3f abc = ab.duplicate().cross(ac);
        Vector3f acd = ac.duplicate().cross(ad);
        Vector3f adb = ad.duplicate().cross(ab);

        if (sameDirection(abc, ao) || abc.lengthSquared()==0) {
            points.clear();
            points.addAll(aP, bP, cP);
            return triangle(direction);
        }
        if (sameDirection(acd, ao)) {
            points.clear();
            points.addAll(aP, cP, dP);
            return triangle(direction);
        }
        if (sameDirection(adb, ao)) {
            points.clear();
            points.addAll(aP, dP, bP);
            return triangle(direction);
        }

        return true;
    }

    private boolean triangle(Vector3f direction) {
        GjkPoint aP = points.get(0);
        GjkPoint bP = points.get(1);
        GjkPoint cP = points.get(2);

        Vector3f a = aP.getPoint();
        Vector3f b = bP.getPoint();
        Vector3f c = cP.getPoint();

        Vector3f ab = b.duplicate().sub(a);
        Vector3f ac = c.duplicate().sub(a);
        Vector3f ao = a.duplicate().negate();

        Vector3f bc = b.duplicate().sub(c);
        Vector3f bo = b.duplicate().negate();

        Vector3f abc = ab.duplicate().cross(ac);

        if (sameDirection(abc.duplicate().cross(ac), ao) || abc.lengthSquared()==0) {
            if (sameDirection(ac, ao)) {
                points.clear();
                points.addAll(aP, cP);
                direction.copy(ac.duplicate().cross(ao).cross(ac));
            } else {
                points.clear();
                points.addAll(aP, bP);
                return lineOld(direction);
            }
        } else {
            if (sameDirection(ab.duplicate().cross(abc), ao)) {
                points.clear();
                points.addAll(aP, bP);
                return lineOld(direction);
            } else {
                if (sameDirection(abc, ao)) {
                    direction.copy(abc);
                } else {
                    points.clear();
                    points.addAll(aP, cP, bP);
                    direction.copy(abc.duplicate().negate());
                }
            }
        }


        return false;
    }

    public boolean line(Vector3f direction) {
        GjkPoint aP = points.get(0);
        GjkPoint bP = points.get(1);

        Vector3f a = aP.getPoint();
        Vector3f b = bP.getPoint();

        Vector3f ab = b.duplicate().sub(a);
        Vector3f ao = a.duplicate().negate();

        float dot = ab.dot(ao);

        if (dot>0) {
            if (dot/ab.lengthSquared()<1) {
                direction.copy(ab.duplicate().cross(ao).cross(ab));
            } else {
                points.clear();
                points.addAll(bP);
                direction.copy(b.duplicate().negate());
            }
        } else {
            points.clear();
            points.addAll(aP);
            direction.copy(ao);
        }

        return false;
    }


    public boolean lineOld(Vector3f direction) {
        GjkPoint aP = points.get(0);
        GjkPoint bP = points.get(1);


        Vector3f a = aP.getPoint();
        Vector3f b = bP.getPoint();

        Vector3f ab = b.duplicate().sub(a);
        Vector3f ao = a.duplicate().negate();

        float dot = ab.dot(ao);

        if (dot>0) {
            direction.copy(ab.duplicate().cross(ao).cross(ab));
        } else {
            points.clear();
            points.addAll(aP);
            direction.copy(ao);
        }

        return false;
    }
    private boolean sameDirection(Vector3f direction, Vector3f other) {
        return direction.dot(other) > 0;
    }

    public boolean isSimplexOfDimension(int dimensionAmount) {
        return points.size()==dimensionAmount+1;
    }

    public boolean isPoint() {
        return isSimplexOfDimension(POINT_DIMENSION);
    }

    public boolean isLine() {
        return isSimplexOfDimension(LINE_DIMENSION);
    }

    public boolean isTriangle() {
        return isSimplexOfDimension(TRIANGLE_DIMENSION);
    }

    public boolean isTetrahedron() {
        return isSimplexOfDimension(TETRAHEDRON_DIMENSION);
    }

    public void addFront(GjkPoint supportPoint) {
        points.insertAsFirstElement(supportPoint);
    }

    public PointBag getPoints() {
        return points;
    }

    public GJKSimplex duplicate() {
        return new GJKSimplex(points.clone());
    }

    public boolean alreadyHasPoint(GjkPoint supportPoint) {
        for (GjkPoint point : points) {
            if (point.equals(supportPoint))
                return true;
        }
        return false;
    }
}
