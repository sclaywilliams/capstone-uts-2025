import java.util.ArrayList;

public class World {

    // intrinsics //
    public WorldBoundary worldBoundary;
    public ArrayList<Robot> robots;

    
    // constructors //
    public World(ArrayList<Robot> robots) {
        this.worldBoundary = new WorldBoundary();
        this.robots = robots;
    }

    public World() {
        this.worldBoundary = new WorldBoundary();
    }
    
    
    
    // functions //

    // creates and places robots
    public void createRobots(int rows, int columns) {
        int spacing = 10;
        int margin = 50;
        ArrayList<Robot> robots = new ArrayList<Robot>();
        int robotId = 0;
        for (int row = 0; row < rows; row++) {
            for (int column = 0; column < columns; column++) {
                Robot robot = new Robot(robotId, margin + spacing * row, margin + spacing * column);
                robots.add(robot);
                robotId++;
            }
        }
        this.robots = robots;
    }

    // basic Euclidean distance calc 
    public double getRobotSeparationDistance(Robot robot1, Robot robot2) {
        return Math.sqrt(Math.pow(robot1.pos_x - robot2.pos_x, 2) + Math.pow(robot1.pos_y - robot2.pos_y, 2));
    }

    // returns [x, y] to the closest boundary
    public double[] getWorldBoundaryDistance(Robot robot, WorldBoundary worldBoundary) {
        double leftDist = robot.pos_x - worldBoundary.min_x;
        double rightDist = robot.pos_x - worldBoundary.max_x;
        double bottomDist = robot.pos_y - worldBoundary.min_y;
        double topDist = robot.pos_y - worldBoundary.max_y;

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
        double commDistance = origin.communicationDistance + 50;
        ArrayList<Robot> localRobots = new ArrayList<Robot>();
        for (Robot robot : robots) {
            if (getRobotSeparationDistance(robot, origin) <= commDistance) {
                localRobots.add(robot);
            }
        }
        return localRobots;
    }
    
    // used to sum vectors of local robots contained within "maxSeparation"
    public double[] getLocalVectorSum(Robot origin, ArrayList<Robot> robots, double maxSeparation) {
        double[] vectorSum = new double[2];
        for (Robot robot : robots) {
            if (getRobotSeparationDistance(origin, robot) <= maxSeparation) {
                vectorSum[0] += origin.pos_x - robot.pos_x;
                vectorSum[1] += origin.pos_y - robot.pos_y;
            }
        }
        
        return vectorSum;
    }
    
    public double[] getWeightedLocalVectorSum(Robot origin, ArrayList<Robot> robots, double maxSeparation) {
        double[] weightedVectorSum = new double[2];

        // local robot influence
        for (Robot robot : robots) {
            double separation = getRobotSeparationDistance(origin, robot);
            if (separation <= maxSeparation) {
                weightedVectorSum[0] += (maxSeparation - separation) * (origin.pos_x - robot.pos_x);
                weightedVectorSum[1] += (maxSeparation - separation) * (origin.pos_y - robot.pos_y);
            }
        }
        // world boundary repulsion
        double[] worldBoundaryDistance = getWorldBoundaryDistance(origin, worldBoundary);
        if (Math.abs(worldBoundaryDistance[0]) <= maxSeparation) {
            weightedVectorSum[0] += (maxSeparation - Math.abs(worldBoundaryDistance[0])) * worldBoundaryDistance[0];
        }
        if (Math.abs(worldBoundaryDistance[1]) <= maxSeparation) {
            weightedVectorSum[1] += (maxSeparation - Math.abs(worldBoundaryDistance[1])) * worldBoundaryDistance[1];
        }

        return weightedVectorSum;
    }

}
