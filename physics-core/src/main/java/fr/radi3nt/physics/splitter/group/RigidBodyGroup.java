package fr.radi3nt.physics.splitter.group;

import fr.radi3nt.physics.constraints.constraint.Constraint;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.ArrayList;
import java.util.List;

public class RigidBodyGroup {

    public final List<RigidBody> group = new ArrayList<>();
    public final List<Constraint> constraints = new ArrayList<>();

}
