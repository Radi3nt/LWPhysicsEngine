package fr.radi3nt.physics.collision;

import fr.radi3nt.physics.collision.contact.manifold.ManifoldPoint;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;
import fr.radi3nt.physics.collision.detection.broad.pre.PreCollisionShape;
import fr.radi3nt.physics.collision.shape.CollisionShapeProvider;
import fr.radi3nt.physics.collision.shape.DuoCollisionShape;
import fr.radi3nt.physics.collision.shape.provider.SetCollisionShapeProvider;
import fr.radi3nt.physics.collision.shape.shapes.CollisionShape;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Predicate;

public class CollisionData {

    private final Collection<PersistentManifold> currentCollisions = new ConcurrentLinkedQueue<>();
    private final Collection<PointFilter> pointsFilter = new ArrayList<>();
    private final Collection<Predicate<RigidBody>> canCollide;

    private CollisionShapeProvider collisionShapeProvider;

    private long currentStep;

    public CollisionData() {
        this.collisionShapeProvider = new SetCollisionShapeProvider(null);
        this.canCollide = new ArrayList<>(Collections.singleton((o) -> false));
    }

    public CollisionData(CollisionShapeProvider collisionShape) {
        this.collisionShapeProvider = collisionShape;
        this.canCollide = new ArrayList<>();
    }

    public CollisionData(CollisionShapeProvider collisionShapeProvider, Predicate<RigidBody> canCollide) {
        this.collisionShapeProvider = collisionShapeProvider;
        this.canCollide = new ArrayList<>(Collections.singleton(canCollide));
    }

    public CollisionData(CollisionShape collisionShape, PreCollisionShape preCollisionShape, Predicate<RigidBody> canCollide) {
        this.collisionShapeProvider = new SetCollisionShapeProvider(preCollisionShape, new DuoCollisionShape(collisionShape, null));
        this.canCollide = new ArrayList<>(Collections.singleton(canCollide));
    }

    public CollisionData(CollisionShape collisionShape, Predicate<RigidBody> canCollide) {
        PreCollisionShape preCollisionShape = collisionShape instanceof PreCollisionShape ? (PreCollisionShape) collisionShape : null;
        this.collisionShapeProvider = new SetCollisionShapeProvider(preCollisionShape, new DuoCollisionShape(collisionShape, null));
        this.canCollide = new ArrayList<>(Collections.singleton(canCollide));
    }

    public CollisionData(CollisionShape collisionShape) {
        PreCollisionShape preCollisionShape = collisionShape instanceof PreCollisionShape ? (PreCollisionShape) collisionShape : null;
        this.collisionShapeProvider = new SetCollisionShapeProvider(preCollisionShape, new DuoCollisionShape(collisionShape, null));
        this.canCollide = new ArrayList<>();
    }

    public CollisionData(CollisionShape collisionShape, PreCollisionShape preCollisionShape) {
        this.collisionShapeProvider = new SetCollisionShapeProvider(preCollisionShape, new DuoCollisionShape(collisionShape, null));
        this.canCollide = new ArrayList<>();
    }

    public void relevance(long step) {
        this.currentStep = step;
    }

    public DuoCollisionShape[] getCollisionShape(RigidBody other) {
        return collisionShapeProvider.getCollisionShapes(other);
    }

    public PreCollisionShape getPreCollisionShape() {
        return collisionShapeProvider.getPreCollisionShape();
    }

    public void setCollisionShapeProvider(CollisionShapeProvider collisionShapeProvider) {
        this.collisionShapeProvider = collisionShapeProvider;
    }

    public CollisionShapeProvider getCollisionShapeProvider() {
        return collisionShapeProvider;
    }

    public Collection<PersistentManifold> getCurrentCollisions() {
        return currentCollisions;
    }

    public Collection<PersistentManifold> getRelevantCollisions() {
        ArrayList<PersistentManifold> relevantManifolds = new ArrayList<>(currentCollisions);
        relevantManifolds.removeIf(manifold -> !manifold.isRelevant(currentStep, 0) || manifold.isEmpty());
        return relevantManifolds;
    }

    public boolean isEmpty() {
        return collisionShapeProvider.isEmpty();
    }

    public boolean cannotCollide(RigidBody other) {
        for (Predicate<RigidBody> rigidBodyPredicate : canCollide) {
            if (!rigidBodyPredicate.test(other))
                return true;
        }
        return false;
    }

    public Collection<PointFilter> getPointsFilter() {
        return pointsFilter;
    }

    public Collection<Predicate<RigidBody>> getCanCollide() {
        return canCollide;
    }

    public interface PointFilter {

        void filter(Collection<ManifoldPoint> points, RigidBody other, long currentStep);

    }
}
