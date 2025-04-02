import java.util.ArrayList;

public class Robot {

    // instance variables //

    private final int id;

    private Vec2D position;
    private Vec2D velocity;
    private Vec2D acceleration;

    private double communicationDistance;
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

    public void move() {
        this.velocity = Vec2D.clampMagnitude(Vec2D.add(velocity, acceleration), speedLimit);
        this.position = Vec2D.add(position, velocity);
    }

    @Override
    public String toString() {
        return "ID: " + this.id + "\n" + "X: " + this.position.getX() + " Y: " + this.position.getY();
    }
}
