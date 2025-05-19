public class Spring {

    // instance variables //

    private final Robot[] robots;

    private final double natLength;
    private final double stiffness;
    private final double damping;

    // constructors //

    public Spring(Robot r1, Robot r2, double natLength, double stiffness, double damping) {
        this.robots = new Robot[2];
        robots[0] = r1;
        robots[1] = r2;

        this.natLength = natLength;
        this.stiffness = stiffness;
        this.damping = damping;
    }

    // getters //

    public Robot[] getRobots() {
        return robots;
    }

    public double getNatLength() {
        return natLength;
    }

    public double getStiffness() {
        return stiffness;
    }

    public double getDamping() {
        return damping;
    }

    // setters //



    // helpers //

    public boolean checkRobots(Robot r1, Robot r2) {
        return robots[0] == r1 && robots[1] == r2 || robots[0] == r2 && robots[1] == r1;
    }

    public Line getLine() {
        return new Line(this.robots[0].getPosition(), this.robots[1].getPosition());
    }



}
