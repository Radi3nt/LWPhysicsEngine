
# Lightweight Physics Engine (LWPhysicsEngine)

This project aims at creating a realtime physics engine to be used in games

## Features

- Different types of integration
    - Euler
        - Semi implicit
        - Implicit
    - Runge–Kutta 4
- Collision detection
    - Shapes
        - Square
        - Cube
        - Capsule
        - Triangles
    - Broadphase
        - Sweeping
        - OBBs and AABBs
        - Bounding Sphere
    - Narrowphase
        - Separating axis theorem
        - Gilbert–Johnson–Keerthi algorithm
- Manifold generation
    - GJK/SAT manifold generation
    - Persistent manifold
- Collision response
    - Separation/resting/collision contact handling
    - Collision contact:
        - Bounce factor
        - Static/kinetic friction factor
    - Resting contact
        - Prevents further penetration
        - Applies friction
        - Constrainted merged in global constraint solver
- Constraints
    - Island splitting
    - Gauss-Seidel SLE solver
    - Support arbitrary constraints
- Sleeping
    - Objects are put to sleep if not moving for a while, allowing skipping collision detection
## Optimizations

Sparse matrices multiplcation algorithm were implemented to make constraint solving as fast possible.  
Currently, the main bottleneck is collision detection.

## Roadmap

- More shapes support
- Optimizing collision detection even further


## Screenshots

![App Screenshot](https://media.discordapp.net/attachments/514492449121632288/1261663461234966549/collision-demo.gif?ex=6693c729&is=669275a9&hm=269551d9c89afda6b8d4a6ce189e6dc91782e136bf8a4c75e88d4b07f0c5ccd8&=)

![App Screenshot](https://media.discordapp.net/attachments/1022994064309747833/1251177812749189162/image.png?ex=66933464&is=6691e2e4&hm=8650b903382341f3a29e7a708aedca35218361d3270bb5f4a8d6b9b484e0b29d&=&format=webp&quality=lossless&width=600&height=337)

![App Screenshot](https://media.discordapp.net/attachments/1022994064309747833/1261450286426226859/image.png?ex=6693a960&is=669257e0&hm=642c6b5cbd8cd052663cd88e02795cb1b318937ba982669ed27a9431e38dbf61&=&format=webp&quality=lossless&width=600&height=330)


## Installation

Install with maven, you'll need one dependency, which you can get by clicking on the link below:

- [JavaUtil](https://github.com/Radi3nt/JavaUtil)


## Usage/Examples

```java
public class PhysicsLogic {

    public static final float DT = 1/120f;

    private final Simulation simulation = new Simulation();
    private boolean stopped;
    private float timeScale = 1;

    public PhysicsLogic() {
    }

    public void start() {
        startPhysicsSimulation();
    }

    private void startPhysicsSimulation() {
        Thread thread = new Thread(() -> {
            float accumulatedTime = 0;

            while (!stopped) {
                long startedTime = System.currentTimeInMillis();
                float timeScale = this.timeScale;
                if (timeScale > 0) {
                    simulation.step(DT * timeScale);
                    accumulatedTime += DT * timeScale;
                }
                long ellapsed = System.currentTimeInMillis() - startedTime;
                if (ellapsed < DT*1_000) {
                    long timeToWait = DT*1_000 - ellapsed;
                    try {
                        Thread.sleep(timeToWait);
                    } catch(Exception e) {
                        e.printStackTrace();
                    }
                }
            }
        }, "Physics thread");
        thread.start();
    }

    public Simulation getSimulation() {
        return simulation;
    }

    public void stop() {
        stopped = true;
    }

    public void setTimeScale(float timeScale) {
        this.timeScale = timeScale;
    }
}

```

And then you can add rigidbodies this way

```java
Matrix3x3 iBIT = ArrayMatrix3x3.newIdentity();
float mass = 1;
iBIT.scale(new SimpleVector3f(1/(12f*mass), 1/(12f*mass), 1/(12f*mass)));
DynamicsProperties dynamicsProperties = new DynamicsProperties(1/mass, iBIT, 1, 1, 1);

BoxShape boxShape = new BoxShape(new SimpleVector3f(1f, 1f, 1f));

simulation.getRigidBodyIsland().add(new RigidBody(0, DynamicsData.zero(dynamicsProperties), new CollisionData(boxShape, null), new NoSleepingData()));
```
## Support

For support, email pro.radi3nt@gmail.com or send a message on discord to @radi3nt.


## Authors

- [@radi3nt](https://github.com/Radi3nt)

