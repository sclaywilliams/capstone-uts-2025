import java.util.ArrayList;

public class World {

    // instance variables //

    private WorldBoundary worldBoundary;
    private ArrayList<Robot> robots;
    private ArrayList<Obstacle> obstacles;
    public int width;
    public int height;
    public int margin;

    // constructors //

    public World(ArrayList<Robot> robots) {
        this.worldBoundary = new WorldBoundary();
        this.robots = robots;
        this.obstacles = new ArrayList<>();
        this.width = 500;
        this.height = 500;
        this.margin = 100;
    }

    public World() {
        this.worldBoundary = new WorldBoundary();
        this.robots = new ArrayList<>();
        this.obstacles = new ArrayList<>();
        this.width = 500;
        this.height = 500;
        this.margin = 100;
    }

    // getters //

    public WorldBoundary getWorldBoundary() {
        return worldBoundary;
    }

    public ArrayList<Robot> getRobots() {
        return robots;
    }

    public ArrayList<Obstacle> getObstacles() {
        return obstacles;
    }

    // setters //

    public void setWorldBoundary(WorldBoundary worldBoundary) {
        this.worldBoundary = worldBoundary;
    }

    public void setObstacles(ArrayList<Obstacle> obstacles) {
        this.obstacles = obstacles;
    }

    public void addObstacle(Obstacle obstacle) {
        obstacles.add(obstacle);
    }

    public void addRobot(Robot robot) {
        robots.add(robot);
    }

    // helpers //

    // creates and places robots
    public void createRobots(int numRobots, int rows, int columns, int spacing, int margin, boolean randomNudge) {
        ArrayList<Robot> robots = new ArrayList<>();
        int robotId = 0;
        for (int row = 0; row < rows; row++) {
            for (int column = 0; column < columns; column++) {
                if (numRobots > 0 && robots.size() > numRobots) {
                    break;
                }
                int randomNudgeValue = randomNudge ? (int) (Math.random() * 10) : 0;
                Robot robot = new Robot(robotId, margin + spacing * row + randomNudgeValue, margin + spacing * column + randomNudgeValue);
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
        double naturalSpringLength = Variables.SPRING_LENGTH;
        double springStiffness = Variables.SPRING_STIFFNESS;
        double springDamping = Variables.SPRING_DAMPING;

        double signalSpringLength = calculateMaxSpringSignalDistance(
                Variables.ACCEPTABLE_SIGNAL_STRENGTH,
                Variables.TRANSMISSION_POWER,
                Variables.FREQUENCY,
                Variables.GAIN);
//        System.out.println(signalSpringLength);

        // remove all existing springs //
        for (Robot robot : robots) {
            robot.removeAllSprings();
        }

        // loop through all pairs of robots //
        for (Robot originRobot : robots) {
            for (Robot destinationRobot : robots) {
                // skip same robot //
                if (destinationRobot.getId() == originRobot.getId()) {
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
                    double angle = Vec2D.getAngle(ab, bc, "dot");

                    // remove spring if obtuse angle is found //
                    if (Math.toDegrees(angle) > 90 && Math.toDegrees(angle) < 270) {
                        obtuseAngleFound = true;
                    }
                }
                // if all angles acute, add spring //
                if (!obtuseAngleFound) {
                    Spring spring = new Spring(
                            originRobot,
                            destinationRobot,
                            naturalSpringLength,
//                            signalSpringLength / 10,
                            springStiffness,
                            springDamping
                    );
                    // check if spring intersects object //
                    Line springLine = spring.getLine();
                    if (this.checkObstacleIntersection(springLine)) {
                        if (!Variables.USE_E2VSM) {
                            continue;
                        }
                        double transmissionPower = Variables.TRANSMISSION_POWER;
                        double totalGain = Variables.GAIN;
                        double frequency = Variables.FREQUENCY;

                        if (calculatePathSignalStrength(springLine, transmissionPower, frequency, totalGain) < Variables.ACCEPTABLE_SIGNAL_STRENGTH) {
                            continue;
                        }
                        //continue; // temp solution //
                    }

                    // add connecting spring to robots //
                    originRobot.addSpring(spring);
                    destinationRobot.addSpring(spring);
                }
            }
        }
    }

    public Vec2D getVerticalBoundaryDistance(Robot robot) {
        double posX = robot.getPosX();


        return new Vec2D(0, 0);

    }

    /**
     * getMinimumObstacleDistance
     *
     * @param robot robot to check distances
     * @return distance to nearest obstacle
     */
    public double getMinimumObstacleDistance(Robot robot) {
        double minDistance = Double.MAX_VALUE;
        for (Obstacle obstacle : obstacles) {
            for (Line edge : obstacle.getEdges()) {
                double distance = edge.getDistanceToPoint(robot.getPosition());
                if (distance < minDistance) {
                    minDistance = distance;
                }
            }
        }
        return minDistance;
    }

    /**
     * getFutureMinimumObstacleDistance
     *
     * @param robot robot to check future movement distances
     * @return distance to nearest obstacle
     */
    public double getFutureMinimumObstacleDistance(Robot robot, Vec2D force) {
        double minDistance = Double.MAX_VALUE;
        Vec2D futurePosition = robot.getFuturePosition(force);
        for (Obstacle obstacle : obstacles) {
            for (Line edge : obstacle.getEdges()) {
                double distance = edge.getDistanceToPoint(futurePosition);
                if (distance < minDistance) {
                    minDistance = distance;
                }
            }
        }
        return minDistance;
    }

    public Vec2D getClosestIntersectionVector(Line line) {
        Vec2D closestIntersectionVector = null;

        for (Obstacle obstacle : obstacles) {
            for (Line edge : obstacle.getEdges()) {
                if (!line.intersects(edge)) {
                    continue;
                }

                Vec2D intersectionPoint = line.findIntersectionPoint(edge);
                Line intersectionLine = new Line(line.getStart(), intersectionPoint);

                if (closestIntersectionVector == null || intersectionLine.getLength() < closestIntersectionVector.getLength()) {
                    closestIntersectionVector = intersectionLine.getVec();
                }
            }
        }
        return closestIntersectionVector;
    }

    public boolean checkIfContainedByObstacle(Vec2D point) {
        for (Obstacle obstacle : obstacles) {
            if (obstacle.isEnclosed() && obstacle.contains(point)) {
                return true;
            }
        }
        return false;
    }

    public boolean checkObstacleIntersection(Line line) {
        for (Obstacle obstacle : obstacles) {
            if (obstacle.intersects(line)) {
                return true;
            }
        }
        return false;
    }

    public boolean checkFutureSignalStrength(Robot robot, Vec2D force) {
        Vec2D futurePosition = robot.getFuturePosition(force);
        double predictedSignalStrength = calculateFutureMaximumSignalStrengthFromRobot(robot, futurePosition);

        return predictedSignalStrength >= Variables.ACCEPTABLE_SIGNAL_STRENGTH;
    }

    public double calculateMaximumSignalStrength(Vec2D point) {
        double maxSignalStrength = -Double.MAX_VALUE;

        double transmissionPower = Variables.TRANSMISSION_POWER;
        double totalGain = Variables.GAIN;
        double frequency = Variables.FREQUENCY;

        for (Robot robot : robots) {
            Line path = new Line(point, robot.getPosition());
            double signalStrength = calculatePathSignalStrength(path, transmissionPower, frequency, totalGain);
            if (path.getLength() == 0) {
                signalStrength = -30; // default max value if right on transmitter //
            }
            if (signalStrength > maxSignalStrength) {
                maxSignalStrength = signalStrength;
            }
        }
        return maxSignalStrength;
    }

    public double calculateFutureMaximumSignalStrengthFromRobot(Robot robot, Vec2D futurePosition) {
        double maxSignalStrength = -Double.MAX_VALUE;

        double transmissionPower = Variables.TRANSMISSION_POWER;
        double totalGain = Variables.GAIN;
        double frequency = Variables.FREQUENCY;

        for (Robot neighbour : robots) {
            if (neighbour.getId() == robot.getId()) {
                continue;
            }
            Line path = new Line(neighbour.getPosition(), futurePosition);
            double signalStrength = calculatePathSignalStrength(path, transmissionPower, frequency, totalGain);
            if (signalStrength > maxSignalStrength) {
                maxSignalStrength = signalStrength;
            }
        }
        return maxSignalStrength;
    }

    public double calculatePathSignalStrength(Line path, double transmissionPower, double frequency, double totalGain) {
        double attenuation = 0;

        for (Obstacle obstacle : obstacles) {
            for (Line edge : obstacle.getEdges()) {
                if (edge.intersects(path)) {
                    attenuation += Utils.getAttenuation(obstacle.getMaterial());
                }
            }
        }
        double freeSpacePathLoss = Utils.calculateFSPL(path.getLength(), frequency, totalGain);

//        System.out.println("PathLoss: " + freeSpacePathLoss);
//        System.out.println("Attenuation: " + attenuation);
//        System.out.println("Transmission power: " + transmissionPower);
//        System.out.println("Signal: " + (transmissionPower - freeSpacePathLoss - attenuation) + "\n");

        return transmissionPower - freeSpacePathLoss - attenuation;
    }

    public double calculateMaxSpringSignalDistance(double minimumAcceptableSignal, double transmissionPower, double frequency, double totalGain) {
        double maxAllowedLoss = transmissionPower - minimumAcceptableSignal;
        return Utils.calculateMaxDistanceFromFSPL(maxAllowedLoss, frequency, totalGain);
    }


}
