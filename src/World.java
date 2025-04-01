import java.util.ArrayList;

public class World {

    // instance variables //

    private WorldBoundary worldBoundary;
    private ArrayList<Robot> robots;
    
    // constructors //

    public World(ArrayList<Robot> robots) {
        this.worldBoundary = new WorldBoundary();
        this.robots = robots;
    }

    public World() {
        this.worldBoundary = new WorldBoundary();
    }

    // getters //

    public WorldBoundary getWorldBoundary() {
        return worldBoundary;
    }

    public ArrayList<Robot> getRobots() {
        return robots;
    }

    // setters //

    public void setWorldBoundary(WorldBoundary worldBoundary) {
        this.worldBoundary = worldBoundary;
    }
    
    // helpers //

    // creates and places robots
    public void createRobots(int rows, int columns, int spacing, int margin) {
        ArrayList<Robot> robots = new ArrayList<Robot>();
        int robotId = 0;
        for (int row = 0; row < rows; row++) {
            for (int column = 0; column < columns; column++) {
                int randomNudge = (int) (Math.random() * 10);
//                randomNudge = 0;
                Robot robot = new Robot(robotId, margin + spacing * row + randomNudge, margin + spacing * column + randomNudge);
                robots.add(robot);
                robotId++;
            }
        }
        this.robots = robots;
    }

    // basic Euclidean distance calc 
    public double getRobotSeparationDistance(Robot robot1, Robot robot2) {
        return Math.sqrt(Math.pow(robot1.getPosX() - robot2.getPosX(), 2) + Math.pow(robot1.getPosY() - robot2.getPosY(), 2));
    }

    public Vec2D getSeparationVector(Robot robot1, Robot robot2) {
        return new Vec2D(robot2.getPosX() - robot1.getPosX(), robot2.getPosY() - robot1.getPosY());
    }

    // returns [x, y] to the closest boundary
    public double[] getWorldBoundaryDistance(Robot robot, WorldBoundary worldBoundary) {
        double leftDist = robot.getPosX() - worldBoundary.getMinX();
        double rightDist = robot.getPosX() - worldBoundary.getMaxX();
        double bottomDist = robot.getPosY() - worldBoundary.getMinY();
        double topDist = robot.getPosY() - worldBoundary.getMaxY();

        double[] worldBoundaryDistance = new double[2];
        if (Math.abs(leftDist) < Math.abs(rightDist)) {
            worldBoundaryDistance[0] = leftDist;
        } else {
            worldBoundaryDistance[0] = rightDist;
        }

        if (Math.abs(bottomDist) < Math.abs(topDist)) {
            worldBoundaryDistance[1] = bottomDist;
        } else {
            worldBoundaryDistance[1] = topDist;
        }

        return worldBoundaryDistance;
    }

    public ArrayList<Robot> getLocalRobots(Robot origin, ArrayList<Robot> robots) {
        double commDistance = origin.getCommunicationDistance();
        ArrayList<Robot> localRobots = new ArrayList<Robot>();
        for (Robot robot : robots) {
            if (getRobotSeparationDistance(robot, origin) <= commDistance) {
                if (origin.getPosX() != robot.getPosX() || origin.getPosY() != robot.getPosY()) {
                    localRobots.add(robot);
                }
            }
        }
        return localRobots;
    }
    
    // used to sum vectors of local robots contained within "communicationDistance"
    public double[] getLocalVectorSum(Robot origin, ArrayList<Robot> robots, double communicationDistance) {
        double[] vectorSum = new double[2];
        for (Robot robot : robots) {
            if (getRobotSeparationDistance(origin, robot) <= communicationDistance) {
                vectorSum[0] += origin.getPosX() - robot.getPosX();
                vectorSum[1] += origin.getPosY() - robot.getPosY();
            }
        }
        
        return vectorSum;
    }
    
    public double[] getWeightedLocalVectorSum(Robot origin, ArrayList<Robot> robots, double communicationDistance) {
        double[] weightedVectorSum = new double[2];

        // local robot influence
        for (Robot robot : robots) {
            double separation = getRobotSeparationDistance(origin, robot);
            if (separation <= communicationDistance) {
                weightedVectorSum[0] += (communicationDistance - separation) * (origin.getPosX() - robot.getPosX());
                weightedVectorSum[1] += (communicationDistance - separation) * (origin.getPosY() - robot.getPosY());
            }
        }
        // world boundary repulsion
        double[] worldBoundaryDistance = getWorldBoundaryDistance(origin, worldBoundary);
        if (Math.abs(worldBoundaryDistance[0]) <= communicationDistance) {
            weightedVectorSum[0] += (communicationDistance - Math.abs(worldBoundaryDistance[0])) * worldBoundaryDistance[0];
        }
        if (Math.abs(worldBoundaryDistance[1]) <= communicationDistance) {
            weightedVectorSum[1] += (communicationDistance - Math.abs(worldBoundaryDistance[1])) * worldBoundaryDistance[1];
        }

        return weightedVectorSum;
    }

    public void buildSpringMesh(ArrayList<Robot> robots) {
        // spring constants (for initial testing) //
        double naturalSpringLength = 100.0;
        double springStiffness = 0.1;
        double springDamping = 0.3;

        // remove all existing springs //
        for (Robot robot : robots) {
            robot.removeAllSprings();
        }

        // loop through all pairs of robots //
        for (Robot originRobot : robots) {
            for (Robot destinationRobot : robots) {
                // skip same robot //
                if (destinationRobot == originRobot) {
                    continue;
                }

                // skip if spring is already built //
                if (originRobot.checkSprings(destinationRobot)) {
                    continue;
                }

                boolean obtuseAngleFound = false;

                // perform acute angle test //
                for (Robot middleRobot : robots) {
                    // skip same robots //
                    if (middleRobot == originRobot || middleRobot == destinationRobot) {
                        continue;
                    }

                    // get vector angle //
                    Vec2D ab = getSeparationVector(originRobot, middleRobot);
                    Vec2D bc = getSeparationVector(destinationRobot, middleRobot);
                    double angle = Vec2D.getAngle(ab, bc);

//                    System.out.println("\nOrigin: " + originRobot);
//                    System.out.println("Destination: " + destinationRobot);
//                    System.out.println("Middle: " + middleRobot);
//
//                    System.out.println("Angle (radians): " + angle);
//                    System.out.println("Angle (degrees): " + Math.toDegrees(angle));

                    // remove spring if obtuse angle is found //
                    if (Math.toDegrees(angle) > 90 && Math.toDegrees(angle) < 270) {
//                        originRobot.removeSpring(destinationRobot);
//                        destinationRobot.removeSpring(originRobot);
//                        System.out.println("Removed:");
//                        System.out.println(originRobot + "\n" + destinationRobot + "\n-----");
                        obtuseAngleFound = true;
                        continue;
                    }
//                    System.out.println("Angle (radians): " + angle);
//                    System.out.println("Angle (degrees): " + Math.toDegrees(angle));
                }
                // if all angles acute, add spring //
                if (!obtuseAngleFound) {
                    Spring spring = new Spring(
                            originRobot,
                            destinationRobot,
                            naturalSpringLength,
                            springStiffness,
                            springDamping
                    );
                    originRobot.addSpring(spring);
                    destinationRobot.addSpring(spring);
//                    System.out.println("Added Spring: ");
//                    System.out.println(originRobot + "\n" + destinationRobot + "\n-----");
                }
            }
        }
    }

    public Vec2D getVerticalBoundaryDistance(Robot robot) {
        double posX = robot.getPosX();


        return new Vec2D(0, 0);

    }

}
