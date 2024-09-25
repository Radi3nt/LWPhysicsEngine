package fr.radi3nt.physics.main;

import java.util.Collections;
import java.util.List;
import java.util.concurrent.AbstractExecutorService;
import java.util.concurrent.TimeUnit;

public class SynchronousExecutorService extends AbstractExecutorService {

    private volatile boolean shutdown;

    public void shutdown() {
        shutdown = true;
    }

    public List<Runnable> shutdownNow() {
        return Collections.emptyList();
    }

    public boolean isShutdown() {
        return shutdown;
    }

    public boolean isTerminated() {
        return shutdown;
    }

    public boolean awaitTermination(long time, TimeUnit unit) {
        return true;
    }

    public void execute(Runnable runnable) {
        runnable.run();
    }

}