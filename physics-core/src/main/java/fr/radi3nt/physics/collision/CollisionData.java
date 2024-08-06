package fr.radi3nt.physics.collision;

import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;
import fr.radi3nt.physics.collision.shape.CollisionShapeProvider;
import fr.radi3nt.physics.collision.shape.DuoCollisionShape;
import fr.radi3nt.physics.collision.detection.broad.pre.PreCollisionShape;
import fr.radi3nt.physics.collision.shape.provider.SetCollisionShapeProvider;
import fr.radi3nt.physics.collision.shape.shapes.CollisionShape;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.ArrayList;
import java.util.Collection;

public class CollisionData {

    private final CollisionShapeProvider collisionShapeProvider;
    private final Collection<PersistentManifold> currentCollisions = new ArrayList<>();
    private long currentStep;
    private boolean empty = false;

    public CollisionData(CollisionShapeProvider collisionShape) {
        this.collisionShapeProvider = collisionShape;
    }

    public CollisionData(CollisionShape collisionShape, PreCollisionShape preCollisionShape) {
        this.collisionShapeProvider = new SetCollisionShapeProvider(preCollisionShape, new DuoCollisionShape(collisionShape, preCollisionShape));
    }

    public CollisionData(CollisionShape collisionShape) {
        PreCollisionShape preCollisionShape = collisionShape instanceof PreCollisionShape ? (PreCollisionShape) collisionShape : null;
        this.collisionShapeProvider = new SetCollisionShapeProvider(preCollisionShape, new DuoCollisionShape(collisionShape, preCollisionShape));
    }

    public CollisionData() {
        this.collisionShapeProvider = new SetCollisionShapeProvider(null);
        empty = true;
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
        return empty;
    }
}
