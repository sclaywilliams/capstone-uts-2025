public class Line {


    private final Vec2D start;
    private final Vec2D end;

    public Line() {
        start = new Vec2D(0, 0);
        end = new Vec2D(0, 0);
    }

    public Line(Vec2D start, Vec2D end) {
        this.start = start;
        this.end = end;
    }

    // getters //

    public Vec2D getStart() {
        return start;
    }

    public Vec2D getEnd() {
        return end;
    }

    // helpers //

    public double getLength() {
        return Math.sqrt(Math.pow(end.getX() - start.getX(), 2) + Math.pow(end.getY() - start.getY(), 2));
    }

    public double getSlope() {
        return (end.getY() - start.getY()) / (end.getX() - start.getX());
    }

}
