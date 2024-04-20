package fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.relative;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.CollisionObject;
import fr.radi3nt.physics.collision.contact.ContactPair;
import fr.radi3nt.physics.collision.contact.manifold.ManifoldPoint;
import fr.radi3nt.physics.collision.detection.narrow.sat.NormalSatPointDetector;
import fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.ManifoldPointBuilder;
import fr.radi3nt.physics.collision.shape.sat.shape.SatShapeObject;
import fr.radi3nt.physics.core.TransformedObject;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class RelativeReuseManifoldPointBuilder implements ManifoldPointBuilder {

    private final RelativeSatCollisionDetector collisionDetector;
    private final NormalSatPointDetector satPointDetector = new NormalSatPointDetector();
    private final float penetrationEpsilon;

    public RelativeReuseManifoldPointBuilder(RelativeSatCollisionDetector collisionDetector, float penetrationEpsilon) {
        this.collisionDetector = collisionDetector;
        this.penetrationEpsilon = penetrationEpsilon;
    }

    @Override
    public List<ManifoldPoint> computeManifolds(ContactPair pair, SatShapeObject sa, SatShapeObject sb) {
        return createContactPoints(pair, sa, sb, collisionDetector.getRelativeMinimumTranslationAxis(), collisionDetector.getLocalVerticesA(), collisionDetector.getRelativeVerticesB());
    }

    private List<ManifoldPoint> createContactPoints(ContactPair pair, SatShapeObject sa, SatShapeObject sb, Vector3f directedNormal, Vector3f[] verticesA, Vector3f[] verticesB) {
        List<ManifoldPoint> pairResults = new ArrayList<>();

        Collection<Vector3f> contactPointsB = sa.getClipPlanes().clipUsingBspace(pair.objectA, pair.objectB, sb.getClipEdges());

        AddToPairResultsA(pair.objectA, pair.objectB, directedNormal, verticesA, contactPointsB, pairResults);

        Collection<Vector3f> contactPointsA = sb.getClipPlanes().clipUsingBspace(pair.objectB, pair.objectA, sa.getClipEdges());

        Vector3f negatedDirectedNormal = directedNormal.duplicate().negate();

        AddToPairResultsB(pair.objectA, pair.objectB, negatedDirectedNormal, verticesB, contactPointsA, pairResults);

        return pairResults;
    }

    private void AddToPairResultsB(CollisionObject objectA, CollisionObject objectB, Vector3f directedNormal, Vector3f[] verticesFirst, Collection<Vector3f> contactPointsAtOpposite, List<ManifoldPoint> pairResults) {
        for (Vector3f contactPointAtOpposite : contactPointsAtOpposite) {
            Vector3f worldContact = objectA.toWorldSpace(contactPointAtOpposite);
            float penetration = satPointDetector.getPenetration(contactPointAtOpposite, directedNormal, verticesFirst);

            if (penetration<penetrationEpsilon)
                continue;

            Vector3f contactPointAtFirst = contactPointAtOpposite.duplicate().add(directedNormal.duplicate().mul(penetration));

            pairResults.add(new ManifoldPoint(contactPointAtFirst, objectB.toLocalSpace(worldContact)));
        }
    }

    private void AddToPairResultsA(TransformedObject objectA, TransformedObject objectB, Vector3f directedNormal, Vector3f[] verticesFirst, Collection<Vector3f> contactPointsAtOpposite, List<ManifoldPoint> pairResults) {
        for (Vector3f contactPointAtOpposite : contactPointsAtOpposite) {
            Vector3f worldContact = objectB.toWorldSpace(contactPointAtOpposite);
            Vector3f aSpace = objectA.toLocalSpace(worldContact);
            float penetration = satPointDetector.getPenetration(aSpace, directedNormal, verticesFirst);

            if (penetration<penetrationEpsilon)
                continue;

            Vector3f contactPointAtFirst = aSpace.duplicate().add(directedNormal.duplicate().mul(penetration));

            pairResults.add(new ManifoldPoint(contactPointAtFirst, contactPointAtOpposite));
        }
    }
}
