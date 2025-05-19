public class Statistics {

    public double averageDistanceToRobot;
    public double averageSignalStrength;
    public double totalCoverage;



    public Statistics(World world) {
        averageDistanceToRobot = calculateAverageDistanceToRobot(world);
        averageSignalStrength = calculateAverageSignalStrength(world);
        totalCoverage = calculateTotalCoverage(world);
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

                if (world.calculateMaximumSignalStrength(pixelPosition) > Variables.MINIMUM_SIGNAL_STRENGTH) {
                    coverage++;
                }
                totalAreaCount++;
            }
        }
        return totalAreaCount > 0 ? coverage / totalAreaCount : 0.0;
    }

    @Override
    public String toString() {
        return ("Statistics: \n" +
               "Average Distance to Robot: " + averageDistanceToRobot + "m \n" +
                "Average Signal Strength: " + averageSignalStrength + " dB \n" +
                "Total Coverage: " + totalCoverage * 100 + "% \n"
        );
    }

}
