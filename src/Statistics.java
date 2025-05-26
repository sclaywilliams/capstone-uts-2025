public class Statistics {

    public boolean isCalculated = false;

    public World world;
    public int iterations;

    public double averageDistanceToRobot;
    public double averageSignalStrength;
    public double totalCoverage;
    public double averageSpringLength;
    public double totalExploration;


    public Statistics() {}

    public Statistics(World world, int iterations) {
        this.world = world;
        this.iterations = iterations;

        calculateStatistics(world);
        this.isCalculated = true;
    }

    public void calculateStatistics(World world) {
        double totalDistance = 0;
        double totalSignalStrength = 0;
        double coverage = 0;
        double exploration = 0;

        double totalSpringLength = 0;
        double springCount = 0;

        double count = 0;
        double signalCount = 0;

        for (int i = world.margin; i < world.margin + world.width; i++) {
            for (int j = world.margin; j < world.margin + world.height; j++) {
                Vec2D pixelPosition = new Vec2D(i, j);
                if (world.checkIfContainedByObstacle(pixelPosition)) {
                    continue;
                }

                // exploration flag //
                boolean isVisible = false;

                // distance, spring length, and exploration //
                double minDistance = Double.MAX_VALUE;
                for (Robot robot : world.getRobots()) {
                    // spring length //
                    for (Spring spring : robot.getSprings()) {
                        totalSpringLength += spring.getLength();
                        springCount++;
                    }
                    // distance //
                    double distance = Utils.getDistanceBetweenPoints(robot.getPosition(), pixelPosition);
                    if (distance < minDistance) {
                        minDistance = distance;
                    }
                    // exploration //
                    if (!world.checkObstacleIntersection(new Line(pixelPosition, robot.getPosition()))) {
                        isVisible = true;
                    }
                }

                if (isVisible) {
                    exploration++;
                }

                totalDistance += minDistance;

                double pixelSignalStrength = world.calculateMaximumSignalStrength(pixelPosition);

                // signal strength, capped to remove invalid values //
                if (pixelSignalStrength < 100) {
                    totalSignalStrength += pixelSignalStrength;
                    signalCount++;
                }

                // coverage //
                if (pixelSignalStrength >= Variables.ACCEPTABLE_SIGNAL_STRENGTH) {
                    coverage++;
                }


                // total area //
                count++;
            }
        }
        this.averageDistanceToRobot = count > 0 ? totalDistance / count : 0.0;
        this.averageSignalStrength = count > 0 ? totalSignalStrength / signalCount : 0.0;
        this.totalCoverage = count > 0 ? coverage / count : 0.0;
        this.averageSpringLength = springCount > 0 ? (totalSpringLength / springCount) / Variables.SPRING_LENGTH : 0.0;
        this.totalExploration = count > 0 ? exploration / count : 0.0;
    }

    public double calculateAverageDistanceToRobot(World world) {
        double totalDistance = 0;
        double count = 0;
        for (int i = world.margin; i < world.margin + world.width; i++) {
            for (int j = world.margin; j < world.margin + world.height; j++) {
                double minDistance = Double.MAX_VALUE;
                Vec2D pixelPosition = new Vec2D(i, j);
                if (world.checkIfContainedByObstacle(pixelPosition)) {
                    continue;
                }

                // get distance to the closest robot //
                for (Robot robot : world.getRobots()) {
                    double distance = Utils.getDistanceBetweenPoints(robot.getPosition(), pixelPosition);
                    if (distance < minDistance) {
                        minDistance = distance;
                    }
                }
                totalDistance += minDistance;
                count++;
            }
        }
        return count > 0 ? totalDistance / count : 0.0;
    }

    public double calculateAverageSignalStrength(World world) {
        double totalSignalStrength = 0;
        double count = 0;

        for (int i = world.margin; i < world.margin + world.width; i++) {
            for (int j = world.margin; j < world.margin + world.height; j++) {
                Vec2D pixelPosition = new Vec2D(i, j);

                if (world.checkIfContainedByObstacle(pixelPosition)) {
                    continue;
                }

                totalSignalStrength += world.calculateMaximumSignalStrength(pixelPosition);
                count++;
            }
        }
        return count > 0 ? totalSignalStrength / count : 0.0;
    }

    public double calculateTotalCoverage(World world) {
        double coverage = 0;
        double totalAreaCount = 0;

        for (int i = world.margin; i < world.margin + world.width; i++) {
            for (int j = world.margin; j < world.margin + world.height; j++) {
                Vec2D pixelPosition = new Vec2D(i, j);

                if (world.checkIfContainedByObstacle(pixelPosition)) {
                    continue;
                }

                if (world.calculateMaximumSignalStrength(pixelPosition) > 100) {
                    coverage++;
                }
                totalAreaCount++;
            }
        }
        return totalAreaCount > 0 ? coverage / totalAreaCount : 0.0;
    }

    @Override
    public String toString() {
        return (
                "Number of Robots: " + world.getRobots().size() + "\n" +
                "Iterations: " + iterations + "\n" +
                "----------------------------\n" +
                "Average Distance to Robot: " + Math.round(averageDistanceToRobot * 100.0) / 100.0 + " m \n" +
                "Average Signal Strength: " + Math.round(averageSignalStrength * 100.0) / 100.0 + " dB \n" +
                "Total Coverage: " + Math.round(totalCoverage * 10000.0) / 100.0 + " % \n" +
                "Average Spring Length: " + Math.round((averageSpringLength / Variables.SPRING_LENGTH) * 100.0) / 100.0 + "\n" +
                "Total Exploration: " + Math.round(totalExploration * 10000.0) / 100.0 + " % \n"
        );
    }

}
