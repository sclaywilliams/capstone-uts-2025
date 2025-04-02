public class Vec2D {
    // instance variables //

    private double x, y;
    private double[] val; // val == [ x, y ] //


    // constructors //

    public Vec2D() {
        this.x = 0;
        this.y = 0;
        this.val = new double[2];
    }

    public Vec2D(double x, double y) {
        this.x = x;
        this.y = y;

        this.val = new double[2];
        this.val[0] = x;
        this.val[1] = y;
    }

    // getters //

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double[] getVal() {
        return val;
    }

    // setters //

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setVal(double[] val) {
        if (val.length != 2) {
            throw new IllegalArgumentException("Vector length must be 2");
        }
        this.val = val;
    }

    // helper functions //

    public double getLength() {
        return Math.sqrt(Math.pow(this.x, 2) + Math.pow(this.y, 2));
    }

    public Vec2D getUnitVector() {
        double length = getLength();
        return new Vec2D(this.x / length, this.y / length);
    }

    // static utility functions //

    public static Vec2D add(Vec2D v1, Vec2D v2) {
        return new Vec2D(v1.getX() + v2.getX(), v1.getY() + v2.getY());
    }

    public static Vec2D subtractVectors(Vec2D v1, Vec2D v2) {
        return new Vec2D(v1.getX() - v2.getX(), v1.getY() - v2.getY());
    }

    public static Vec2D multiplyVectors(Vec2D v1, Vec2D v2) {
        return new Vec2D(v1.getX() * v2.getX(), v1.getY() * v2.getY());
    }

    public static Vec2D multiplyMagnitude(Vec2D v1, double magnitude) {
        return new Vec2D(v1.getX() * magnitude, v1.getY() * magnitude);
    }

    public static Vec2D clampMagnitude(Vec2D v1, double magnitude) {
        return multiplyMagnitude(v1.getUnitVector(), magnitude);
    }

    public static double dot(Vec2D v1, Vec2D v2) {
        return v1.getX() * v2.getX() + v1.getY() * v2.getY();
    }

    public static double getAngle(Vec2D v1, Vec2D v2) {
        return Math.acos(dot(v1, v2)/(v1.getLength()*v2.getLength()));
    }

    @Override
    public String toString() {
        return "Vec2D{" + "x=" + x + ", y=" + y + "} | Magnitude: " + getLength();
    }

}
