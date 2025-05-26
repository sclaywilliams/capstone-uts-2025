import jdk.jshell.Snippet;

import javax.swing.*;
import java.awt.geom.Line2D;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Map;

public class Main {

    // main //
    public static void main(String[] args) {

        boolean runTests = true;

        if (runTests) {
            try {
                runTestProgram();
            } catch (Exception e) {
                e.printStackTrace();
            }
        } else {
            // empty, basic, medium, complex, maximum //
            String worldType = "complex";
            boolean showStatistics = true;
            boolean showHeatmap = true;
            int iterations = 2000;
            int robotMargin = 10;
            boolean randomNudge = false;
            String material = "concrete";

            World world = new World();
            world.createRobots(-1, 5, 10, 10, world.margin + robotMargin, randomNudge);
            //            world.createRobots(-1, 15, 10, 10, world.margin + robotMargin, randomNudge);
            //        world.createRobots(36, 6, 6, 10, world.margin + robotMargin, randomNudge);
            ArrayList<Robot> robots = world.getRobots();

            // World Boundary //
            ArrayList<Vec2D> points = new ArrayList<>();
            points.add(new Vec2D(world.margin, world.margin));
            points.add(new Vec2D(world.margin + world.width, world.margin));
            points.add(new Vec2D(world.margin + world.width, world.margin + world.height));
            points.add(new Vec2D(world.margin, world.margin + world.height));
            Obstacle worldBoundary = new Obstacle(points);
            world.addObstacle(worldBoundary);

            switch (worldType) {
                case "basic":
                    world.addObstacle(new Obstacle(new Vec2D(275, 275), 50, 50, "rectangle", false, material));
                    world.addObstacle(new Obstacle(new Vec2D(375, 275), 50, 50, "rectangle", false, material));
                    world.addObstacle(new Obstacle(new Vec2D(275, 375), 50, 50, "rectangle", false, material));
                    world.addObstacle(new Obstacle(new Vec2D(375, 375), 50, 50, "rectangle", false, material));
                    break;

                case "medium":
                    // stars
                    world.addObstacle(new Obstacle(new Vec2D(300, 250), 60, 80, "star", false, material));
                    world.addObstacle(new Obstacle(new Vec2D(450, 350), 120, 160, "star", false, material));
                    world.addObstacle(new Obstacle(new Vec2D(300, 450), 90, 120, "star", false, material));
                    // triangles
                    world.addObstacle(new Obstacle(new Vec2D(500, 200), 50, 50, "triangle", false, material));
                    world.addObstacle(new Obstacle(new Vec2D(500, 500), 50, -50, "triangle", false, material));
                    world.addObstacle(new Obstacle(new Vec2D(200, 370), 50, 50, "triangle", false, material));
                    break;

                case "complex":
                    // row 1
                    world.addObstacle(new Obstacle(new Vec2D(300, 300), 100, 100, "office2", true, material));
                    world.addObstacle(new Obstacle(new Vec2D(400, 300), 100, -100, "office2", true, material));
                    world.addObstacle(new Obstacle(new Vec2D(500, 300), 100, -100, "office1", true, material));
                    // row 2
                    world.addObstacle(new Obstacle(new Vec2D(300, 450), 100, -100, "office2", true, material));
                    world.addObstacle(new Obstacle(new Vec2D(400, 450), -100, -100, "office1", true, material));
                    world.addObstacle(new Obstacle(new Vec2D(500, 450), -100, 100, "office1", true, material));
                    break;
                case "maximum":
                    // edges //

                    // left edge
                    world.addObstacle(new Obstacle(new Vec2D(130, 295), 60, -50, "office3", true, material));
                    world.addObstacle(new Obstacle(new Vec2D(130, 350), 60, -60, "office3", true, material));
                    world.addObstacle(new Obstacle(new Vec2D(130, 410), 60, -60, "office3", true, material));
                    world.addObstacle(new Obstacle(new Vec2D(130, 470), 60, -60, "office3", true, material));
                    // bottom left corner
                    world.addObstacle(new Obstacle(new Vec2D(130, 550), 60, -100, "office3", true, material));
                    // bottom row
                    world.addObstacle(new Obstacle(new Vec2D(190, 575), 60, -50, "office1", true, material));
                    world.addObstacle(new Obstacle(new Vec2D(245, 575), 50, -50, "office1", true, material));
                    world.addObstacle(new Obstacle(new Vec2D(295, 575), 50, -50, "office1", true, material));
                    world.addObstacle(new Obstacle(new Vec2D(345, 575), 50, -50, "office1", true, material));
                    world.addObstacle(new Obstacle(new Vec2D(395, 575), 50, -50, "office1", true, material));
                    world.addObstacle(new Obstacle(new Vec2D(445, 575), 50, -50, "office1", true, material));
                    world.addObstacle(new Obstacle(new Vec2D(495, 575), 50, -50, "office1", true, material));
                    // bottom right corner
                    world.addObstacle(new Obstacle(new Vec2D(560, 575), 80, -50, "office1", true, material));
                    // right edge
                    world.addObstacle(new Obstacle(new Vec2D(575, 525), -50, -50, "office3", true, material));
                    world.addObstacle(new Obstacle(new Vec2D(575, 475), -50, -50, "office3", true, material));
                    world.addObstacle(new Obstacle(new Vec2D(575, 425), -50, -50, "office3", true, material));
                    world.addObstacle(new Obstacle(new Vec2D(575, 375), -50, -50, "office3", true, material));
                    world.addObstacle(new Obstacle(new Vec2D(575, 325), -50, -50, "office3", true, material));
                    world.addObstacle(new Obstacle(new Vec2D(575, 275), -50, -50, "office3", true, material));
                    world.addObstacle(new Obstacle(new Vec2D(575, 225), -50, -50, "office3", true, material));
                    // top right corner
                    world.addObstacle(new Obstacle(new Vec2D(540, 150), 120, 100, "office1", true, material));
                    // top row
                    world.addObstacle(new Obstacle(new Vec2D(455, 125), 50, 50, "office1", true, material));
                    world.addObstacle(new Obstacle(new Vec2D(405, 125), 50, 50, "office1", true, material));
                    world.addObstacle(new Obstacle(new Vec2D(355, 125), 50, 50, "office1", true, material));
                    world.addObstacle(new Obstacle(new Vec2D(305, 125), 50, 50, "office1", true, material));

                    // middle section //

                    // bottom left corner
                    world.addObstacle(new Obstacle(new Vec2D(230, 480), -50, 50, "office1", true, material));
                    // bottom row
                    world.addObstacle(new Obstacle(new Vec2D(280, 480), -50, 50, "office1", true, material));
                    world.addObstacle(new Obstacle(new Vec2D(330, 480), -50, 50, "office1", true, material));
                    world.addObstacle(new Obstacle(new Vec2D(380, 480), -50, 50, "office1", true, material));
                    world.addObstacle(new Obstacle(new Vec2D(430, 480), -50, 50, "office1", true, material));
                    // bottom right corner
                    world.addObstacle(new Obstacle(new Vec2D(480, 480), 50, -50, "office3", true, material));
                    // right edge
                    world.addObstacle(new Obstacle(new Vec2D(465, 430), 80, -50, "office3", true, material));
                    world.addObstacle(new Obstacle(new Vec2D(465, 380), 80, -50, "office3", true, material));
                    world.addObstacle(new Obstacle(new Vec2D(465, 330), 80, -50, "office3", true, material));
                    // custom top right corner
                    ArrayList<Vec2D> lP = new ArrayList<>();

                    // break room
                    world.addObstacle(new Obstacle(new Vec2D(315, 375), 220, -160, "office2", true, material));


                    break;

                case "empty":
                default:
                    break;
            }

            // line test //
            //        Line testLine = new Line(new Vec2D(1, 3), new Vec2D(4, 1));
            //        Vec2D testPoint = new Vec2D(0, 0);
            //        System.out.println("Distance: " + testLine.getDistanceToPoint(testPoint));

            // timer //
            long startTime = System.currentTimeMillis();

            // move robots //
            //        virtualSpringMesh(robots, world, 100);
            extendedVirtualSpringMesh(robots, world, iterations, runTests);

            // statistics //
            long endTime = System.currentTimeMillis();
            long elapsedTime = endTime - startTime;
            System.out.println("Elapsed time: " + elapsedTime + " ms");
            Statistics statistics = new Statistics();
            if (showStatistics) {
                statistics = new Statistics(world, iterations);
                System.out.println(statistics);
            }

            // draw robots //
            JFrame frame = new JFrame();
            frame.setSize(600, 630);
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.getContentPane().add(new VisualOutput(world, showHeatmap, statistics));
            frame.setVisible(true);
        }
    }

    // testing and results //

    public static void runTestProgram() throws IOException {
        boolean showStatistics = true;
        boolean showHeatmap = true;
        int iterations = 0;
        int robotMargin = 10;
        boolean randomNudge = false;
        String material = "concrete";
        Variables.USE_E2VSM = true;

        String[] worldTypes = {"empty", "basic", "medium", "complex", "maximum"};
        int[] robotRows = {5, 10, 15, 20};

        // create and open file //
        BufferedWriter writer = new BufferedWriter(new FileWriter("statistics_initial_state_e2vsm.txt"));

        for (String worldType : worldTypes) {
            for (int numRows : robotRows) {

                World world = new World();
                world.createRobots(-1, numRows, 10, 10, world.margin + robotMargin, randomNudge);
                ArrayList<Robot> robots = world.getRobots();

                // World Boundary //
                ArrayList<Vec2D> points = new ArrayList<>();
                points.add(new Vec2D(world.margin, world.margin));
                points.add(new Vec2D(world.margin + world.width, world.margin));
                points.add(new Vec2D(world.margin + world.width, world.margin + world.height));
                points.add(new Vec2D(world.margin, world.margin + world.height));
                Obstacle worldBoundary = new Obstacle(points);
                world.addObstacle(worldBoundary);

                switch (worldType) {
                    case "basic":
                        world.addObstacle(new Obstacle(new Vec2D(275, 275), 50, 50, "rectangle", false, material));
                        world.addObstacle(new Obstacle(new Vec2D(375, 275), 50, 50, "rectangle", false, material));
                        world.addObstacle(new Obstacle(new Vec2D(275, 375), 50, 50, "rectangle", false, material));
                        world.addObstacle(new Obstacle(new Vec2D(375, 375), 50, 50, "rectangle", false, material));
                        break;

                    case "medium":
                        // stars
                        world.addObstacle(new Obstacle(new Vec2D(300, 250), 60, 80, "star", false, material));
                        world.addObstacle(new Obstacle(new Vec2D(450, 350), 120, 160, "star", false, material));
                        world.addObstacle(new Obstacle(new Vec2D(300, 450), 90, 120, "star", false, material));
                        // triangles
                        world.addObstacle(new Obstacle(new Vec2D(500, 200), 50, 50, "triangle", false, material));
                        world.addObstacle(new Obstacle(new Vec2D(500, 500), 50, -50, "triangle", false, material));
                        world.addObstacle(new Obstacle(new Vec2D(200, 370), 50, 50, "triangle", false, material));
                        break;

                    case "complex":
                        // row 1
                        world.addObstacle(new Obstacle(new Vec2D(300, 300), 100, 100, "office2", true, material));
                        world.addObstacle(new Obstacle(new Vec2D(400, 300), 100, -100, "office2", true, material));
                        world.addObstacle(new Obstacle(new Vec2D(500, 300), 100, -100, "office1", true, material));
                        // row 2
                        world.addObstacle(new Obstacle(new Vec2D(300, 450), 100, -100, "office2", true, material));
                        world.addObstacle(new Obstacle(new Vec2D(400, 450), -100, -100, "office1", true, material));
                        world.addObstacle(new Obstacle(new Vec2D(500, 450), -100, 100, "office1", true, material));
                        break;
                    case "maximum":
                        // edges //

                        // left edge
                        world.addObstacle(new Obstacle(new Vec2D(130, 295), 60, -50, "office3", true, material));
                        world.addObstacle(new Obstacle(new Vec2D(130, 350), 60, -60, "office3", true, material));
                        world.addObstacle(new Obstacle(new Vec2D(130, 410), 60, -60, "office3", true, material));
                        world.addObstacle(new Obstacle(new Vec2D(130, 470), 60, -60, "office3", true, material));
                        // bottom left corner
                        world.addObstacle(new Obstacle(new Vec2D(130, 550), 60, -100, "office3", true, material));
                        // bottom row
                        world.addObstacle(new Obstacle(new Vec2D(190, 575), 60, -50, "office1", true, material));
                        world.addObstacle(new Obstacle(new Vec2D(245, 575), 50, -50, "office1", true, material));
                        world.addObstacle(new Obstacle(new Vec2D(295, 575), 50, -50, "office1", true, material));
                        world.addObstacle(new Obstacle(new Vec2D(345, 575), 50, -50, "office1", true, material));
                        world.addObstacle(new Obstacle(new Vec2D(395, 575), 50, -50, "office1", true, material));
                        world.addObstacle(new Obstacle(new Vec2D(445, 575), 50, -50, "office1", true, material));
                        world.addObstacle(new Obstacle(new Vec2D(495, 575), 50, -50, "office1", true, material));
                        // bottom right corner
                        world.addObstacle(new Obstacle(new Vec2D(560, 575), 80, -50, "office1", true, material));
                        // right edge
                        world.addObstacle(new Obstacle(new Vec2D(575, 525), -50, -50, "office3", true, material));
                        world.addObstacle(new Obstacle(new Vec2D(575, 475), -50, -50, "office3", true, material));
                        world.addObstacle(new Obstacle(new Vec2D(575, 425), -50, -50, "office3", true, material));
                        world.addObstacle(new Obstacle(new Vec2D(575, 375), -50, -50, "office3", true, material));
                        world.addObstacle(new Obstacle(new Vec2D(575, 325), -50, -50, "office3", true, material));
                        world.addObstacle(new Obstacle(new Vec2D(575, 275), -50, -50, "office3", true, material));
                        world.addObstacle(new Obstacle(new Vec2D(575, 225), -50, -50, "office3", true, material));
                        // top right corner
                        world.addObstacle(new Obstacle(new Vec2D(540, 150), 120, 100, "office1", true, material));
                        // top row
                        world.addObstacle(new Obstacle(new Vec2D(455, 125), 50, 50, "office1", true, material));
                        world.addObstacle(new Obstacle(new Vec2D(405, 125), 50, 50, "office1", true, material));
                        world.addObstacle(new Obstacle(new Vec2D(355, 125), 50, 50, "office1", true, material));
                        world.addObstacle(new Obstacle(new Vec2D(305, 125), 50, 50, "office1", true, material));

                        // middle section //

                        // bottom left corner
                        world.addObstacle(new Obstacle(new Vec2D(230, 480), -50, 50, "office1", true, material));
                        // bottom row
                        world.addObstacle(new Obstacle(new Vec2D(280, 480), -50, 50, "office1", true, material));
                        world.addObstacle(new Obstacle(new Vec2D(330, 480), -50, 50, "office1", true, material));
                        world.addObstacle(new Obstacle(new Vec2D(380, 480), -50, 50, "office1", true, material));
                        world.addObstacle(new Obstacle(new Vec2D(430, 480), -50, 50, "office1", true, material));
                        // bottom right corner
                        world.addObstacle(new Obstacle(new Vec2D(480, 480), 50, -50, "office3", true, material));
                        // right edge
                        world.addObstacle(new Obstacle(new Vec2D(465, 430), 80, -50, "office3", true, material));
                        world.addObstacle(new Obstacle(new Vec2D(465, 380), 80, -50, "office3", true, material));
                        world.addObstacle(new Obstacle(new Vec2D(465, 330), 80, -50, "office3", true, material));
                        // custom top right corner
                        ArrayList<Vec2D> lP = new ArrayList<>();

                        // break room
                        world.addObstacle(new Obstacle(new Vec2D(315, 375), 220, -160, "office2", true, material));


                        break;

                    case "empty":
                    default:
                        break;
                }

                ArrayList<Statistics> statsArray = extendedVirtualSpringMesh(robots, world, iterations, true);

                String algorithm = Variables.USE_E2VSM ? "E2VSM" : "EVSM";

                writer.write("Algorithm: " + algorithm + " | World Type: " + worldType + " | Robots: " + (numRows * 10) + '\n');
                writer.write("iterations, averageDistanceToRobot, averageSignalStrength, averageSpringLength, totalCoverage, totalExploration\n");
                for (Statistics stats : statsArray) {
                    writer.write(stats.iterations + "," +
                                 stats.averageDistanceToRobot + "," +
                                 stats.averageSignalStrength + "," +
                                 stats.averageSpringLength + "," +
                                 stats.totalCoverage + "," +
                                 stats.totalExploration + "\n"
                                );
                }
                writer.write("\n");
                System.out.println("World Type: " + worldType + " | Robots: " + (numRows * 10) + " | Logged\n");
            }
        }
        writer.close();
    }

    // robot algorithms //

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
     * @param robots list of robots
     * @param world environment to move around in
     * @param iterations max number of iterations before exiting the program (ideally equilibrium is reached prior)
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

//                Vec2D selfOrgForce = robot.calculateSelfOrganisingForce();
//                robot.addAcceleration(selfOrgForce);

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
//            System.out.println("Entropy: " + entropy);
//            System.out.println("EntropyVector: " + entropyVector);

            if (entropy < 5000) {
                System.out.println("Reached Equilibrium at " + i + " iterations");
                return;
            }
        }
        System.out.println("Did not reach equilibrium after " + iterations + " iterations...");
    }

    /** extendedVirtualSpringMesh
     *
     * @param robots list of all robots
     * @param world simulation environment
     * @param iterations number of iterations in the simulation
     */

    public static ArrayList<Statistics> extendedVirtualSpringMesh(ArrayList<Robot> robots, World world, int iterations, boolean runTests) {
        /** pseudocode
         * calculate distance and bearing to neighbours
         * calculate if edge or interior robot
         * if edge: apply Fexpl or Fexpn as driving force Fd
         * calculate Fsof
         * calculate total force: F = Fsof + Fd
         * calculate distance to closest obstacle Dobs
         * if Dobs > Dmin: apply force as F
         * else if Dobs <= Dmin: apply force as -F
         */

        ArrayList<Statistics> statsArray = new ArrayList<>();

        for (int i = 0; i < iterations; i++) {
            // calculate spring mesh //
            world.buildSpringMesh(robots);

            if (i % 100 == 0) {
//                System.out.println("Stats at " + i + " iterations");
//                System.out.println(new Statistics(world, iterations));
                System.out.println((int) (((double) i / (double) iterations) * 100) + "%");
                if (runTests) {
                    statsArray.add(new Statistics(world, i));
                }
            }

            double totalVelocity = 0.0;


            for (Robot robot : robots) {
                // calculate distance and bearing to neighbours


                // calculate if edge or interior robot
//                boolean isEdge = robot.calculateEdge();
                double[] sweepAngles = robot.calculateEdge();
                boolean isEdge = sweepAngles[0] != -1;
//                boolean isEdge = false;


                // if edge: apply fExpl or fExpn as driving force Fd - else: apply 0 force
                Vec2D drivingForce = isEdge ? robot.calculateExplorationForce(world, sweepAngles) : new Vec2D();
//                Vec2D drivingForce = isEdge ? robot.calculateExpansionForce() : new Vec2D(); // todo

                // calculate Fsof
                Vec2D selfOrganisingForce = robot.calculateSelfOrganisingForce();

                // calculate total force
                Vec2D totalForce = Vec2D.add(selfOrganisingForce, drivingForce);

                // calculate distance to closest obstacle
                double obstacleDistance = world.getFutureMinimumObstacleDistance(robot, totalForce);
                boolean willMaintainConnection = Variables.USE_E2VSM ? world.checkFutureSignalStrength(robot, totalForce) : true;

                // if Dobs > Dmin: apply force as F
                if (obstacleDistance > Variables.MIN_OBSTACLE_DISTANCE && willMaintainConnection) {
                    robot.addAcceleration(totalForce);
//                    robot.move();
                }
                // else if Dobs <= Dmin: apply force as -F
                else {
                    Vec2D inverseForce = Vec2D.multiplyMagnitude(totalForce, -1);
                    // validate movement path to avoid double negative movement pathing //
                    boolean validMovementPath = robot.checkMovementPath(inverseForce, world);
                    if (validMovementPath) {
                        robot.addAcceleration(inverseForce);
                    } else {
                        robot.addAcceleration(totalForce);
//                        continue;
                    }

//                    robot.moveWithObstacleCheck(world);
                }
                totalVelocity += robot.move();
            }
            double averageVelocity = totalVelocity / robots.size();
//            System.out.println("Average velocity: " + averageVelocity);
            if (averageVelocity < Variables.MINIMUM_ROBOT_MOVE_THRESHOLD) {
//                System.out.println("Reached equilibrium at " + i + " iterations");
                return statsArray;
            }
        }
        // final state //
        if (runTests) {
            statsArray.add(new Statistics(world, iterations));
        }
//        System.out.println("Failed to reach equilibrium after " + iterations + " iterations");
        // done //
        return statsArray;
    }


    // helpers //

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