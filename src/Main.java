import javax.swing.*;
import java.awt.event.KeyEvent;
import java.util.ArrayList;
import java.io.InputStream;
import java.util.TimerTask;
import java.util.Timer;

public class Main {

    // main //
    public static void main(String[] args) {
        World world = new World();
        world.createRobots(5, 5, 50, 200);
        ArrayList<Robot> robots = world.getRobots();

        // move robots //
        virtualSpringMesh(robots, world, 1000);

        // draw robots //
        JFrame frame = new JFrame();
        frame.setSize(600, 630);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.getContentPane().add(new VisualOutput(world));
        frame.setVisible(true);
    }

    // robot movement functions //

    /** moveVectorSum
     * - (initial algorithm)
     * - uses inverse vectors from communication distance circle to push the robot away from other local robots
     */
    public static void moveVectorSum(ArrayList<Robot> robots, World world, int iterations) {
        for (int i = 0; i < iterations; i++) {
            for (Robot robot : robots) {
                double maxMovement = robot.getCommunicationDistance() / 2;
                double[] vectorSum = world.getWeightedLocalVectorSum(robot, robots, maxMovement);
                double vectorMagnitude = Math.sqrt(Math.pow(vectorSum[0], 2) + Math.pow(vectorSum[1], 2));

                // calculate movement //
                if (vectorMagnitude > 0) {
                    robot.setVelocity(new Vec2D(
                            (maxMovement / vectorMagnitude) * vectorSum[0],
                            (maxMovement / vectorMagnitude) * vectorSum[1]));
                    robot.move();
                }

                // boundary clamping //
                if (robot.getPosX() < world.getWorldBoundary().getMinX()) {
                    robot.setPosX(world.getWorldBoundary().getMinX());
                }
                if (robot.getPosY() < world.getWorldBoundary().getMinY()) {
                    robot.setPosY(world.getWorldBoundary().getMinY());
                }
                if (robot.getPosX() > world.getWorldBoundary().getMaxX()) {
                    robot.setPosX(world.getWorldBoundary().getMaxX());
                }
                if (robot.getPosY() > world.getWorldBoundary().getMaxY()) {
                    robot.setPosY(world.getWorldBoundary().getMaxY());
                }
            }
        }
    }

    /** virtualSpringMesh
     * @param robots - list of robots
     * @param world - environment to move around in
     * @param iterations - max number of iterations before exiting the program (ideally equilibrium is reached prior)
     */
    public static void virtualSpringMesh(ArrayList<Robot> robots, World world, int iterations) {
        // VSM Equation //
        // accel = (stiffness * (currentSpringLength − naturalSpringLength) * unitVec(originRobot -> connectedRobot)) − (damping * velocity)

        for (int i = 0; i < iterations; i++) {
            double entropy = 0.0;
//            Vec2D entropyVector = new Vec2D(0.0, 0.0);

            world.buildSpringMesh(robots);
            for (Robot robot : robots) {
                ArrayList<Robot> localRobots = world.getLocalRobots(robot, robots);

                for (Robot localRobot : localRobots) {
                    // spring force //
                    Vec2D springForce = calculateSpringForce(robot, localRobot, world);
                    robot.addAcceleration(springForce);

                    // TODO: world boundary force
                    // obstacle avoidance //
                    Vec2D worldBoundaryForce = calculateWorldBoundaryForce(robot, world);

                    // calculate total entropy //
                    entropy += springForce.getLength();
//                    entropyVector = Vec2D.add(entropyVector, springForce);
                }
                robot.move();
            }
            System.out.println("Entropy: " + entropy);
//            System.out.println("EntropyVector: " + entropyVector);

            if (entropy < 5000) {
                System.out.println("Reached Equilibrium at " + iterations + " iterations");
                return;
            }
        }
        System.out.println("Did not reach equilibrium after " + iterations + " iterations...");
    }

    public static Vec2D calculateSpringForce(Robot r1, Robot r2, World world) {
        Spring spring = r1.findSpring(r2);
        if (spring == null) {
            // if no spring, don't change acceleration //
            return new Vec2D(0, 0);
        }

        Vec2D separationVector = world.getSeparationVector(r1, r2);
        double springForce = spring.getStiffness() * (separationVector.getLength() - spring.getNatLength());
        Vec2D dampingForce = Vec2D.multiplyMagnitude(r1.getVelocity(), spring.getDamping());
        Vec2D acceleration = Vec2D.multiplyMagnitude(separationVector, springForce);
        acceleration = Vec2D.subtractVectors(acceleration, dampingForce);
        return acceleration;
    }

    public static Vec2D calculateWorldBoundaryForce(Robot robot, World world) {
        Vec2D verticalForce = new Vec2D(0, 0);



        Vec2D horizontalForce = new Vec2D(0, 0);



        return new Vec2D(0, 0);

    }

}