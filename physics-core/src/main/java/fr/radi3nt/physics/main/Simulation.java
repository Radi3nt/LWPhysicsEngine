package fr.radi3nt.physics.main;

import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.contact.cache.ContactPairCacheProvider;
import fr.radi3nt.physics.collision.contact.cache.HashPersistentManifoldCache;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;
import fr.radi3nt.physics.collision.detection.CollisionDetection;
import fr.radi3nt.physics.collision.detection.broad.BroadPhaseStrategies;
import fr.radi3nt.physics.collision.detection.broad.aabb.AABBBroadPhaseDetectionStrategy;
import fr.radi3nt.physics.collision.detection.broad.aabb.shapes.ParentAABBPreCollisionShape;
import fr.radi3nt.physics.collision.detection.broad.sphere.SphereBroadPhaseDetectionStrategy;
import fr.radi3nt.physics.collision.detection.generators.DimensionalContactPairCacheProvider;
import fr.radi3nt.physics.collision.detection.generators.generator.BroadphaseOrderingPairGenerator;
import fr.radi3nt.physics.collision.detection.generators.generator.PairGenerator;
import fr.radi3nt.physics.collision.detection.generators.provider.AABBOneDimensionProvider;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.gjk.GJKNarrowPhaseDetectionAlgorithm;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.gjk.shapes.GjkProcessedShape;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.gjk.shapes.provider.GjkBoxShapeProvider;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.gjk.shapes.provider.GjkCapsuleShapeProvider;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.gjk.shapes.provider.GjkSphereShapeProvider;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.gjk.shapes.provider.GjkTriangleShapeProvider;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.SATNarrowPhaseDetectionAlgorithm;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.provider.SatBoxProcessedShapeProvider;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.provider.SatSquareProcessedShapeProvider;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.provider.SatTriangleProcessedShapeProvider;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape.SatProcessedShape;
import fr.radi3nt.physics.collision.detection.narrow.dispacher.SupportedCollisionDispatcher;
import fr.radi3nt.physics.collision.detection.narrow.processed.CachingProcessedShapeProvider;
import fr.radi3nt.physics.collision.detection.narrow.processed.MapProcessedShapeProvider;
import fr.radi3nt.physics.collision.detection.narrow.processed.ProcessedShapeProvider;
import fr.radi3nt.physics.collision.detection.predicate.broadphase.BroadPhaseDetectionStrategyPredicate;
import fr.radi3nt.physics.collision.response.CompositionCollisionContactSolver;
import fr.radi3nt.physics.collision.response.constrained.SimpleNoPenetrationConstraintProvider;
import fr.radi3nt.physics.collision.response.constrained.SimultaneousImpulseRestingContactSolver;
import fr.radi3nt.physics.collision.response.sequential.SequentialImpulseCollisionContactSolver;
import fr.radi3nt.physics.collision.shape.shapes.*;
import fr.radi3nt.physics.constraints.constraint.DriftParameters;
import fr.radi3nt.physics.constraints.constraint.list.InstantConstraintList;
import fr.radi3nt.physics.constraints.constraint.list.SetConstraintList;
import fr.radi3nt.physics.constraints.sle.ProjectedGaussSeidelSolver;
import fr.radi3nt.physics.constraints.sle.lambda.SetWarmStartingLambdaProvider;
import fr.radi3nt.physics.constraints.solver.ImpulseConstraintSolver;
import fr.radi3nt.physics.constraints.solver.filled.ListConstraintFiller;
import fr.radi3nt.physics.constraints.solver.mass.InverseMassMatrixComputer;
import fr.radi3nt.physics.constraints.solver.mass.SparseInverseMassMatrixComputer;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.force.caster.*;
import fr.radi3nt.physics.dynamics.island.ArrayListRigidBodyIsland;
import fr.radi3nt.physics.dynamics.island.ListRigidBodyIsland;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;
import fr.radi3nt.physics.dynamics.ode.IntegratorOdeSolver;
import fr.radi3nt.physics.dynamics.ode.OdeSolver;
import fr.radi3nt.physics.dynamics.ode.integrator.SemiImplicitEulerIntegrator;
import fr.radi3nt.physics.splitter.ConstrainedIsland;
import fr.radi3nt.physics.splitter.GroupIslandSplitter;
import fr.radi3nt.physics.splitter.IslandSplitter;
import fr.radi3nt.physics.splitter.group.ConstraintBodyIslandRelationGrouper;

import java.util.*;
import java.util.concurrent.Callable;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.function.Consumer;
import java.util.function.Predicate;

public class Simulation {

    private static final int BATCH_ISLAND_SIZE = 20;

    private static final float DEFAULT_ADDED_SIZE_EPSILON = 1E-3f;
    public static final float GRAVITY = -9.81f * 2;

    private final ListRigidBodyIsland rigidBodyIsland = new ArrayListRigidBodyIsland();
    private final CollisionDetection collisionDetection;
    private final OdeSolver odeSolver;

    private final HashPersistentManifoldCache manifoldCache = new HashPersistentManifoldCache();
    private final ImpulseConstraintSolver impulseConstraintSolver;
    private final IslandSplitter islandSplitter;
    private final CompositionCollisionContactSolver collisionContactSolver;

    public final InstantConstraintList instantConstraintList = new InstantConstraintList();

    public final Collection<Runnable> afterSimulationUpdate = new ConcurrentLinkedQueue<>();
    public final Collection<Runnable> collisionSimulationUpdate = new ConcurrentLinkedQueue<>();
    public final Collection<Runnable> beforeSimulationUpdate = new ConcurrentLinkedQueue<>();

    private final Queue<RigidBody> pendingAdd = new ConcurrentLinkedQueue<>();
    private final Queue<RigidBody> pendingRemove = new ConcurrentLinkedQueue<>();

    private final ExecutorService pool = Executors.newWorkStealingPool(2);
    //private final ExecutorService pool = new SynchronousExecutorService();

    private final FluidDragForceCaster airDrag = new FluidDragForceCaster(new FluidDragForceCaster.DragProperties(1.225f, new SimpleVector3f(0, 0, 0)), new FluidDragForceCaster.DragCoefficientSupplier() {
        @Override
        public float getSurfaceCoefficient(RigidBody body) {
            return 0.3f;
        }

        @Override
        public float getDragCoefficient(RigidBody body) {
            return 0.115f;
        }
    });
    private final TorqueDampForceCaster torqueDrag = new TorqueDampForceCaster(0.001f);
    private final Collection<CachingProcessedShapeProvider<?>> cachingProviders = new ArrayList<>();

    private final ProcessedShapeProvider<SatProcessedShape> satShapeProvider;
    private final MapProcessedShapeProvider<GjkProcessedShape> gjkShapeProvider;

    private long step;
    private double time;

    public Simulation() {
        //odeSolver = new AverageRungeKutta4OdeSolver(new ImplicitEulerIntegrator(), new CompositeForceCaster(new ForceDataForceCaster()));
        //odeSolver = new AverageRungeKutta4OdeSolver(new ImplicitEulerIntegrator(), new CompositeForceCaster(new ForceDataForceCaster(), new SetMassedVectorForceCaster(new SimpleVector3f(0, GRAVITY, 0), new SimpleVector3f()), torqueDrag));
        odeSolver = new IntegratorOdeSolver(new SemiImplicitEulerIntegrator(this::processCollisions), new CompositeForceCaster(new ForceDataForceCaster(), new SetMassedVectorForceCaster(new SimpleVector3f(0, GRAVITY, 0), new SimpleVector3f()), torqueDrag));
        //odeSolver = new AverageRungeKutta4OdeSolver(new ImplicitEulerIntegrator(), new CompositeForceCaster(new AttractionForceCaster(12, new SimpleVector3f())));
        //odeSolver = new IntegrateRungeKutta4OdeSolver(new ImplicitEulerIntegrator(), new CompositeForceCaster(new VectorForceCaster(new SimpleVector3f(0, -9.81f, 0), new SimpleVector3f())));
        //odeSolver = new IntegratorOdeSolver(new ImplicitEulerIntegrator(), new CompositeForceCaster(new VectorForceCaster(new SimpleVector3f(0, -9.81f, 0), new SimpleVector3f())));

        PairGenerator<RigidBody> pairGenerator = new BroadphaseOrderingPairGenerator(new BroadPhaseStrategies(new SphereBroadPhaseDetectionStrategy()));
        ContactPairCacheProvider aabbCacheProvider = new DimensionalContactPairCacheProvider(pairGenerator, new AABBOneDimensionProvider());

        //InverseMassMatrixComputer inverseMassMatrixComputer = new ArrayInverseMassMatrixComputer();
        InverseMassMatrixComputer inverseMassMatrixComputer = new SparseInverseMassMatrixComputer();
        impulseConstraintSolver = new ImpulseConstraintSolver(inverseMassMatrixComputer, () -> new SetWarmStartingLambdaProvider(0.9f), new ProjectedGaussSeidelSolver(1e-3f, 50));
        SimultaneousImpulseRestingContactSolver solver = new SimultaneousImpulseRestingContactSolver(
                new SimpleNoPenetrationConstraintProvider(new DriftParameters(0.1f)),
                instantConstraintList, this);
        collisionContactSolver = new CompositionCollisionContactSolver(new SequentialImpulseCollisionContactSolver(this, 100), solver);
        //collisionContactSolver = new CompositionCollisionContactSolver(solver);

        satShapeProvider = createSatProvider();
        CachingProcessedShapeProvider<SatProcessedShape> satCachingProvider = new CachingProcessedShapeProvider<>(satShapeProvider);
        cachingProviders.add(satCachingProvider);

        Map<Class<? extends CollisionShape>, ProcessedShapeProvider<? extends GjkProcessedShape>> gjkProviders = new HashMap<>();
        gjkProviders.put(BoxShape.class, GjkBoxShapeProvider.INSTANCE);
        gjkProviders.put(CapsuleShape.class, GjkCapsuleShapeProvider.INSTANCE);
        gjkProviders.put(TriangleShape.class, GjkTriangleShapeProvider.INSTANCE);
        gjkProviders.put(SphereShape.class, GjkSphereShapeProvider.INSTANCE);

        gjkShapeProvider = new MapProcessedShapeProvider<>(gjkProviders);

        collisionDetection = new CollisionDetection(aabbCacheProvider, manifoldCache, new SupportedCollisionDispatcher(
                new SATNarrowPhaseDetectionAlgorithm(satCachingProvider),
            new GJKNarrowPhaseDetectionAlgorithm(gjkShapeProvider)
        ), pool);
        islandSplitter = new GroupIslandSplitter(new ConstraintBodyIslandRelationGrouper(instantConstraintList), false);
    }

    public static ProcessedShapeProvider<SatProcessedShape> createSatProvider() {
        final ProcessedShapeProvider<SatProcessedShape> satShapeProvider;
        Map<Class<? extends CollisionShape>, ProcessedShapeProvider<? extends SatProcessedShape>> satProviders = new HashMap<>();
        satProviders.put(BoxShape.class, SatBoxProcessedShapeProvider.INSTANCE);
        satProviders.put(SquareShape.class, SatSquareProcessedShapeProvider.INSTANCE);
        satProviders.put(TriangleShape.class, SatTriangleProcessedShapeProvider.INSTANCE);

        satShapeProvider = new MapProcessedShapeProvider<>(satProviders);
        return satShapeProvider;
    }

    public void step(float dt) {
        while (!pendingAdd.isEmpty()) {
            rigidBodyIsland.add(pendingAdd.poll());
        }
        while (!pendingRemove.isEmpty()) {
            rigidBodyIsland.remove(pendingRemove.poll());
        }

        for (Runnable runnable : beforeSimulationUpdate) {
            runnable.run();
        }

        RigidBodyIsland result = odeSolver.integrate(rigidBodyIsland, dt);
        stepSleeping(result);

        rigidBodyIsland.copyStates(result);
        for (int i = 0; i < rigidBodyIsland.getSize(); i++) {
            RigidBody rigidBody = rigidBodyIsland.getRigidBody(i);
            rigidBody.getCollisionData().relevance(step);
        }

        manifoldCache.cleanup(step);

        for (Runnable runnable : afterSimulationUpdate) {
            runnable.run();
        }

        for (CachingProcessedShapeProvider<?> cachingProvider : cachingProviders) {
            cachingProvider.clearCache();
        }

        instantConstraintList.done();

        time+=dt;
        step++;
    }

    private void processCollisions(RigidBodyIsland result, float dt) {
        Collection<PersistentManifold> process = collisionDetection.process(result, step);

        for (Runnable runnable : collisionSimulationUpdate) {
            runnable.run();
        }

        collisionContactSolver.solve(process, dt);


        ConstrainedIsland[] islands = islandSplitter.getIslands(result);

        //debugConstraints(islands);


        List<Callable<Object>> callables = new ArrayList<>();
        List<ConstrainedIsland> subset = new ArrayList<>();

        for (ConstrainedIsland island : islands) {
            subset.add(island);
            if (subset.size()>=BATCH_ISLAND_SIZE) {
                submit(dt, callables, subset);
                subset = new ArrayList<>();
            }
        }

        if (!subset.isEmpty()) {
            submit(dt, callables, subset);
        }

        try {
            pool.invokeAll(callables);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private static void debugConstraints(ConstrainedIsland[] islands) {
        int mostBodies = 0;
        int mostConstraints = 0;
        for (ConstrainedIsland island : islands) {
            mostBodies = Math.max(island.island.getSize(), mostBodies);
            if (mostConstraints<island.constraints.size()) {
                mostConstraints = island.constraints.size();
            }
        }
        System.out.println("Island amount: " + islands.length + ", most bodies: " + mostBodies + " most constraints: " + mostConstraints);
    }

    private void stepSleeping(RigidBodyIsland result) {
        int sleeping = 0;
        for (int i = 0; i < rigidBodyIsland.getSize(); i++) {
            RigidBody rigidBody = rigidBodyIsland.getRigidBody(i);
            rigidBody.getSleepingData().step(rigidBody.getDynamicsData());
            if (rigidBody.getSleepingData().isSleeping())
                sleeping++;
        }
        System.out.println("Sleeping " + sleeping + "/" + result.getSize());
    }

    private void submit(float dt, List<Callable<Object>> callables, List<ConstrainedIsland> subset) {
        callables.add(() -> {
            for (ConstrainedIsland constrainedIsland : subset) {
                impulseConstraintSolver.solve(new ListConstraintFiller(new SetConstraintList(constrainedIsland.constraints)), constrainedIsland.island, dt);
            }
            return null;
        });
    }

    public void wake(Predicate<RigidBody> predicate) {
        for (int i = 0; i < rigidBodyIsland.getSize(); i++) {
            RigidBody rigidBody = rigidBodyIsland.getRigidBody(i);
            if (predicate.test(rigidBody))
                rigidBody.getSleepingData().wakeUp();
        }
    }

    public void consume(Predicate<RigidBody> predicate, Consumer<RigidBody> consumer) {
        for (int i = 0; i < rigidBodyIsland.getSize(); i++) {
            RigidBody rigidBody = rigidBodyIsland.getRigidBody(i);
            if (predicate.test(rigidBody))
                consumer.accept(rigidBody);
        }
    }

    public void wakeRecursively(RigidBody current, float addedSize) {
        BroadPhaseDetectionStrategyPredicate predicate = new BroadPhaseDetectionStrategyPredicate(AABBBroadPhaseDetectionStrategy.standard(), current, new ParentAABBPreCollisionShape(current.getCollisionData().getPreCollisionShape(), addedSize));

        for (int i = 0; i < rigidBodyIsland.getSize(); i++) {
            RigidBody rigidBody = rigidBodyIsland.getRigidBody(i);
            if (!rigidBody.getSleepingData().isAwoken() && predicate.test(rigidBody)) {
                rigidBody.getSleepingData().wakeUp();
                wakeRecursively(rigidBody, addedSize);
            }
        }
    }

    public void wakeRecursively(RigidBody current) {
        wakeRecursively(current, DEFAULT_ADDED_SIZE_EPSILON);
    }

    public long getStep() {
        return step;
    }

    public ProcessedShapeProvider<SatProcessedShape> getSatShapeProvider() {
        return satShapeProvider;
    }

    public MapProcessedShapeProvider<GjkProcessedShape> getGjkShapeProvider() {
        return gjkShapeProvider;
    }

    public ListRigidBodyIsland getRigidBodyIsland() {
        return rigidBodyIsland;
    }

    public HashPersistentManifoldCache getManifoldCache() {
        return manifoldCache;
    }

    public double getTime() {
        return time;
    }

    public void stop() {
        pool.shutdown();
    }

    public void add(RigidBody rigidBody) {
        pendingRemove.remove(rigidBody);
        pendingAdd.add(rigidBody);
    }

    public void remove(RigidBody rigidBody) {
        pendingAdd.remove(rigidBody);
        pendingRemove.add(rigidBody);
    }

    public float getGravity() {
        return -GRAVITY;
    }
}
