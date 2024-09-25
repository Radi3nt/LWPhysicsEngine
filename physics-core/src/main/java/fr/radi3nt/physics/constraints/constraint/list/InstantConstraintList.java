package fr.radi3nt.physics.constraints.constraint.list;

import fr.radi3nt.physics.constraints.constraint.Constraint;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

public class InstantConstraintList extends SetConstraintList {

    private final Collection<Constraint> stepConstraints = new ArrayList<>();

    @Override
    public void done() {
        stepConstraints.clear();
    }

    public Collection<Constraint> getStepConstraints() {
        return stepConstraints;
    }

    @Override
    public List<Constraint> getConstraints() {
        List<Constraint> constraints = new ArrayList<>(stepConstraints);
        constraints.addAll(super.getConstraints());
        return Collections.unmodifiableList(constraints);
    }
}
