package fr.radi3nt.physics.main;

import java.util.concurrent.atomic.AtomicInteger;

public class RigidBodyIdManager {

    private final AtomicInteger current = new AtomicInteger();

    public int newId() {
        return current.getAndIncrement();
    }

}
