package fr.radi3nt.physics.collision.detection.narrow.sat.computer.part;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.contact.manifold.ManifoldPoint;

import java.util.Collection;

public interface ManifoldPointBuilder {

    Collection<ManifoldPoint> computeManifolds(ShapedPair shapedPair, Vector3f worldSpaceDirectedNormal, float overlap, int normal);

}
