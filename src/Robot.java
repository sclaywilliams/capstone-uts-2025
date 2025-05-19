import java.util.ArrayList;

public class Robot {

    // instance variables //

    private final int id;

    private Vec2D position;
    private Vec2D velocity;
    private Vec2D acceleration;

    private double communicationDistance;
    private double sightDistance;
    private final double speedLimit;

    private ArrayList<Robot> localRobots;
    private ArrayList<Spring> springs;


    // constructors //

    public Robot(int id, double posX, double posY) {
        this.id = id;
        this.position = new Vec2D(posX, posY);
        this.velocity = new Vec2D();
        this.acceleration = new Vec2D();
        this.communicationDistance = 1000;
        this.speedLimit = 5.0;
        this.springs = new ArrayList<>();
        this.sightDistance = 1000;
    }

    // getters //

    public int getId() {
        return id;
    }

    public double getPosX() {
        return position.getX();
    }

    public double getPosY() {
        return position.getY();
    }

    public Vec2D getPosition() {
        return position;
    }

    public Vec2D getVelocity() {
        return velocity;
    }

    public Vec2D getAcceleration() {
        return acceleration;
    }

    public double getCommunicationDistance() {
        return communicationDistance;
    }

    public ArrayList<Robot> getLocalRobots() {
        return localRobots;
    }

    public ArrayList<Spring> getSprings() {
        return springs;
    }

    // setters //

    public void setPosX(double posX) {
        this.position.setX(posX);
    }

    public void setPosY(double posY) {
        this.position.setY(posY);
    }

    public void setPosition(Vec2D pos) {
        this.position = pos;
    }

    public void setVelocity(Vec2D velocity) {
        this.velocity = velocity;
    }

    public void setAcceleration(Vec2D acceleration) {
        this.acceleration = acceleration;
    }

    public void setCommunicationDistance(double communicationDistance) {
        this.communicationDistance = communicationDistance;
    }

    public void setLocalRobots(ArrayList<Robot> localRobots) {
        this.localRobots = localRobots;
    }

    public void setSprings(ArrayList<Spring> springs) {
        this.springs = springs;
    }

    // helpers //

    public void addAcceleration(Vec2D acceleration) {
        this.acceleration = Vec2D.add(acceleration, this.acceleration);
    }

    public void addVelocity(Vec2D velocity) {
        this.velocity = Vec2D.add(velocity, this.velocity);
    }

    public void stopRobot() {
        this.acceleration = new Vec2D();
        this.velocity = new Vec2D();
    }

    public ArrayList<Robot> getNeighbours() {
        ArrayList<Robot> neighbours = new ArrayList<>();
        for (Spring spring : springs) {
            Robot[] springRobots = spring.getRobots();
            for (Robot robot : springRobots) {
                if (robot.getId() != this.getId()) {
                    neighbours.add(robot);
                    break;
                }
            }
        }
        return neighbours;
    }

    public double getDistanceToRobot(Robot robot) {
//        System.out.println(this.getPosition() + " " + robot.getPosition());
        return Math.sqrt(Math.pow(this.getPosX() - robot.getPosX(), 2) + Math.pow(this.getPosY() - robot.getPosY(), 2));
    }

    public Vec2D getVectorToRobot(Robot robot) {
        return new Vec2D(robot.getPosX() - this.getPosX(), robot.getPosY() - this.getPosY());
    }

    // assumption is that only edge robots will be checked
    public Vec2D calculateExplorationForce(World world, double[] sweepAngles) {
        Vec2D explorationForce = new Vec2D();

        // (m * Wi) * Ks * (li - lo) * unitVector - kdx
        // accel = (stiffness * (currentSpringLength − naturalSpringLength) * unitVec(originRobot -> connectedRobot)) − (damping * velocity)


        // get list of visible walls within sweep angle
        ArrayList<Vec2D> vectorsToWalls = this.getLineOfSightVectors(world, (int) sweepAngles[0], (int) sweepAngles[1]);

        double distanceSquareSum = 0;
        for (Vec2D vector : vectorsToWalls) {
            distanceSquareSum += Math.pow(vector.getLength(), 2);
        }

        // for each visible wall
        // weight = distanceToWall * sqrt( 1 / sum(1-k: distanceToWall ^ 2) )
        for (Vec2D vector : vectorsToWalls) {
            double weight = vector.getLength() * Math.sqrt(1 / distanceSquareSum);
            Vec2D weightedVector = Vec2D.multiplyMagnitude(vector.getUnitVector(), weight);
            explorationForce = Vec2D.add(explorationForce, weightedVector);
        }

        return explorationForce;
    }

    // TODO
    public Vec2D calculateExpansionForce() {
        Vec2D force = new Vec2D();

        return force;
    }

    public ArrayList<Vec2D> getLineOfSightVectors(World world, int minAngle, int maxAngle) {
        ArrayList<Vec2D> lineOfSightVectors = new ArrayList<>();

//        System.out.println("minAngle: " + minAngle + ", maxAngle: " + maxAngle);

        for (int angle = minAngle; ; angle++) {
            if (angle >= 360) {
                angle -= 360;
            }

            if (minAngle <= maxAngle) {
                if (angle >= maxAngle) {
                    break;
                }
            } else {
                if (angle > maxAngle && angle < minAngle) {
                    break;
                }
            }

            double theta = Math.toRadians(angle);
            Vec2D mV = new Vec2D(
                    Math.cos(theta),
                    Math.sin(theta)
            );

            Vec2D rayCastPoint = Vec2D.add(this.position, Vec2D.multiplyMagnitude(mV, this.sightDistance));
            Line sightLine = new Line(this.position, rayCastPoint);
            Vec2D lineOfSightVector = world.getClosestIntersectionVector(sightLine);
            if (lineOfSightVector != null) {
                lineOfSightVectors.add(lineOfSightVector);
            }
        }

        return lineOfSightVectors;
    }

    // calculates if robot is an edge, based upon if neighbours are contained within projected bounds //
    public /* boolean */ double[] calculateEdge() {
        ArrayList<Robot> neighbours = getNeighbours();

        if (neighbours.size() <= 2) {
//            return true;
            // return larger than 180º angle //

        }

        for (Robot n1 : neighbours) {
            for (Robot n2 : neighbours) {
                if (n1.getId() == n2.getId()) {
                    continue;
                }


                Line l1 = new Line(this.getPosition(), n1.getPosition());
                Line l2 = new Line(this.getPosition(), n2.getPosition());

                double angle = Line.getAngle(l1, l2, "dot"); // in radians //

                int orientation = Line.orientation(this.getPosition(), n1.getPosition(), n2.getPosition());

                if (orientation == 1) {
                    angle = 2 * Math.PI - angle;
                }

                if (Math.toDegrees(angle) < 90 || Math.toDegrees(angle) > 270) {
                    continue;
                }

                double maxLength = 4 * Math.max(Math.abs(l1.getLength()), Math.abs(l2.getLength()));

                Vec2D i = this.getPosition();
                Vec2D j = n1.getPosition();
                Vec2D k = n2.getPosition();

                double theta = angle / 2; // in radians //

                Vec2D v1 = l1.getVec();
                Vec2D v2 = l2.getVec();
                double x = v1.getX();
                double y = v1.getY();
                Vec2D mV = new Vec2D(
                        x * Math.cos(theta) - y * Math.sin(theta),
                        x * Math.sin(theta) + y * Math.cos(theta)
                );

                Vec2D m = Vec2D.add(i, Vec2D.multiplyMagnitude(mV, 10));

                // i, j, m, k //
                // ^ clockwise order //
                ArrayList<Vec2D> points = new ArrayList<>();
                points.add(i);
                points.add(j);
                points.add(m);
                points.add(k);
                Obstacle checkedArea = new Obstacle(points);

                int containedNeighbours = 0;
                for (Robot n3 : neighbours) {
                    if (n3.getId() == this.getId() || n3.getId() == n1.getId() || n3.getId() == n2.getId()) {
                        continue;
                    }

                    if (checkedArea.contains(n3.getPosition())) {
                        containedNeighbours++;
                    }
                }
                if (containedNeighbours == 0) {
                    // return true for old boolean functionality
//                    return true;
                    double minAngle = Math.atan2(v1.getY(), v1.getX()) + Math.PI;
                    double maxAngle = Math.atan2(v2.getY(), v2.getX()) + Math.PI;
//                    double aM = Math.atan2(mV.getY(), mV.getX()) + Math.PI;
//                    System.out.println("a1: " + Math.toDegrees(a1) + ", maxAngle: " + Math.toDegrees(maxAngle) + ", aM: " + Math.toDegrees(aM));
                    return new double[]{ Math.toDegrees(minAngle), Math.toDegrees(maxAngle) };
                }
            }
        }
        // return false for old boolean functionality
//        return false;
        return new double[] {-1, -1};
    }

    public Vec2D calculateSelfOrganisingForce() {
        ArrayList<Robot> neighbours = getNeighbours();

        /*
         * Rx = received signal strength
         * Tx = transmitter power in dB
         * Lp = path loss
         * L = reference loss constant at 1m
         * N = path loss exponent
         * D = distance in metres in free space loss environment
         *
         */

        // accel = (stiffness * (currentSpringLength − naturalSpringLength) * unitVec(originRobot -> connectedRobot)) − (damping * velocity)

        double Tx = 23.0; // robot transmitter power in dB //
        double L = 40.2; // reference loss constant at 1m //
        double N = 3.3;
        double frequency = 2.45;

        Vec2D selfOrgForce = new Vec2D();

        for (Robot neighbour : neighbours) {

            double D = this.getDistanceToRobot(neighbour);

            double Lp = 33 + N * 10 * Math.log10(D) + 20 * Math.log10(frequency);

            double Rx = Tx - Lp;

            double dirtyExponent = (Tx - Rx - 33 - 20 * Math.log10(frequency)) / (10 * N);

            double signalStrengthLength = Math.pow(10, dirtyExponent);
            Vec2D separationVector = this.getVectorToRobot(neighbour);
            Spring spring = this.findSpring(neighbour);

            double springForce = spring.getStiffness() * (signalStrengthLength - spring.getNatLength());
            Vec2D dampingForce = Vec2D.multiplyMagnitude(this.getVelocity(), spring.getDamping());
            Vec2D acceleration = Vec2D.multiplyMagnitude(separationVector, springForce);
            acceleration = Vec2D.subtractVectors(acceleration, dampingForce);
            selfOrgForce = Vec2D.add(selfOrgForce, acceleration);
        }
        return selfOrgForce;
    }

    // springs //

    public void addSpring(Spring spring) {
        if (!this.springs.contains(spring)) {
            this.springs.add(spring);
        }
    }

    public void removeSpring(Robot robot) {
        boolean removed = springs.removeIf(spring -> spring.checkRobots(this, robot));
        if (removed) {
            stopRobot();
        }
    }

    public void removeAllSprings() {
        this.springs.clear();
        stopRobot();
    }

    public boolean checkSprings(Robot robot) {
        for (Spring spring : springs) {
            if (spring.checkRobots(this, robot)) {
                return true;
            }
        }
        return false;
    }

    public Spring findSpring(Robot robot) {
        for (Spring spring : springs) {
            if (spring.checkRobots(this, robot)) {
                return spring;
            }
        }
        return null;
    }

    public Vec2D getFuturePosition(Vec2D force) {
        Vec2D futureVelocity = Vec2D.clampMagnitude(Vec2D.add(velocity, force), speedLimit);
        return Vec2D.add(this.position, futureVelocity);
    }

    public boolean checkMovementPath(Vec2D force, World world) {
        Vec2D futurePosition = this.getFuturePosition(force);
        Line movementPath = new Line(this.position, futurePosition);
        for (Obstacle obstacle : world.getObstacles()) {
            if (obstacle.intersects(movementPath)) {
                return false;
            }
        }
        return true;
    }

    // returns magnitude of current velocity //
    public double move() {
        this.velocity = Vec2D.clampMagnitude(Vec2D.add(velocity, acceleration), speedLimit);
        this.position = Vec2D.add(position, velocity);
        return this.velocity.getLength();
    }

    public void moveWithObstacleCheck(World world) {
        this.velocity = Vec2D.clampMagnitude(Vec2D.add(velocity, acceleration), speedLimit);
        Line movementPath = new Line(this.position, Vec2D.add(this.position, this.velocity));
        for (Obstacle obstacle : world.getObstacles()) {
            if (obstacle.intersects(movementPath)) {
//                this.position = Vec2D.add(position, Vec2D.multiplyMagnitude(velocity, -1));
                return;
            }
        }
        this.position = Vec2D.add(position, velocity);
    }

    @Override
    public String toString() {
        return "\nID: " + this.id + " " + "X: " + this.position.getX() + " Y: " + this.position.getY();
    }
}
