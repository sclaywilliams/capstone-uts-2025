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

        // build spring mesh //
        world.buildSpringMesh(robots);

        // test vector maths //
//        Vec2D v1 = new Vec2D(1, -2);
//        Vec2D v2 = new Vec2D(-2, 1);
//        double angle = Vec2D.getAngle(v1, v2);
//        System.out.println("Angle (radians): " + angle);
//        System.out.println("Angle (degrees): " + Math.toDegrees(angle));

        // draw robots //
        JFrame frame = new JFrame();
        frame.setSize(600, 630);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.getContentPane().add(new VisualOutput(world));
        frame.setVisible(true);

        // move robots //
//        virtualSpringMesh(robots, world, 10);

        // timer testing //
//        moveVectorSum(robots, world, 1000);
//        Timer timer = new Timer();

//        for (int i = 0; i < 10; i++) {
//        int i = 0;
//        while (i < 10) {
//            TimerTask task = new TimerTask() {
//                @Override
//                public void run() {
//                    System.out.println("Iteration: ");
//                    virtualSpringMesh(robots, world, 1);
//                }
//            };
//            timer.schedule(task, 1000L * i++);
//        }

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
                if (robot.getPosX() < world.getWorldBoundary().minX) {
                    robot.setPosX(world.getWorldBoundary().minX);
                }
                if (robot.getPosY() < world.getWorldBoundary().minY) {
                    robot.setPosY(world.getWorldBoundary().minY);
                }
                if (robot.getPosX() > world.getWorldBoundary().maxX) {
                    robot.setPosX(world.getWorldBoundary().maxX);
                }
                if (robot.getPosY() > world.getWorldBoundary().maxY) {
                    robot.setPosY(world.getWorldBoundary().maxY);
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
        double naturalSpringLength = 100.0;
        double springStiffness = 0.1;
        double springDamping = 0.3;
        //accel = (stiffness * (currentSpringLength − naturalSpringLength) * unitVec(originRobot -> connectedRobot)) − (damping * velocity)

        for (int i = 0; i < iterations; i++) {
            for (Robot robot : robots) {
                ArrayList<Robot> localRobots = world.getLocalRobots(robot, robots);

                for (Robot localRobot : localRobots) {
                    Vec2D separationVector = world.getSeparationVector(robot, localRobot);
                    double springForce = springStiffness * (separationVector.getLength() - naturalSpringLength);
                    Vec2D dampingForce = Vec2D.multiplyMagnitude(robot.getVelocity(), springDamping);
                    Vec2D acceleration = Vec2D.multiplyMagnitude(separationVector, springForce);
                    acceleration = Vec2D.subtractVectors(acceleration, dampingForce);
                    robot.addAcceleration(acceleration);
                }
                robot.move();
            }
//            Thread.sleep(1000);
        }
    }

}