import java.util.ArrayList;

public class Line {

    // instance variables //

    private final Vec2D start;
    private final Vec2D end;
    private final ArrayList<Vec2D> points;

    // constructors //

    public Line(Vec2D start, Vec2D end) {
        this.start = start;
        this.end = end;
        points = buildPoints(start, end);
    }

    private ArrayList<Vec2D> buildPoints(Vec2D start, Vec2D end) {
        ArrayList<Vec2D> points = new ArrayList<>();
        points.add(start);
        points.add(end);
        return points;
    }

    // getters //

    public Vec2D getStart() {
        return start;
    }

    public Vec2D getEnd() {
        return end;
    }

    public ArrayList<Vec2D> getPoints() {
        return points;
    }

    // helpers //

    public Vec2D getVec() {
//        return new Vec2D(Math.abs(end.getX() - start.getX()), Math.abs(end.getY() - start.getY()));
        return new Vec2D(end.getX() - start.getX(), end.getY() - start.getY());
    }

    public double getLength() {
        return Math.sqrt(Math.pow(end.getX() - start.getX(), 2) + Math.pow(end.getY() - start.getY(), 2));
    }

    public double getSlope() {
        return (end.getY() - start.getY()) / (end.getX() - start.getX());
    }

    // returns angle in radians //
    public static double getAngle(Line l1, Line l2, String type) {
        Vec2D v1 = l1.getVec();
        Vec2D v2 = l2.getVec();

        return Vec2D.getAngle(v1, v2, type);
    }

    /**
     * getDistanceToPoint
     * <br/> <br/>
     * calculation grabbed from: <a href="https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment">stack overflow</a>
     *
     * @param point point to calculate for
     * @return distance to point
     */
    public double getDistanceToPoint(Vec2D point) {
        double lengthSquared = Math.pow(getLength(), 2);
        if (lengthSquared == 0) return Utils.getDistanceBetweenPoints(start, point);

        Vec2D v1 = Vec2D.subtractVectors(point, start);
        Vec2D v2 = Vec2D.subtractVectors(end, start);
        double t = Utils.clampDouble(Vec2D.dot(v1, v2) / lengthSquared, 0, 1);

        Vec2D v2Scaled = Vec2D.multiplyMagnitude(v2, t);
        Vec2D projection = Vec2D.add(start, v2Scaled);
        return Utils.getDistanceBetweenPoints(point, projection);
    }

    public boolean intersects(Line line) {

        Vec2D p1 = this.start;
        Vec2D q1 = this.end;
        Vec2D p2 = line.start;
        Vec2D q2 = line.end;


        // Find the four orientations needed for general and
        // special cases
        int o1 = orientation(p1, q1, p2);
        int o2 = orientation(p1, q1, q2);
        int o3 = orientation(p2, q2, p1);
        int o4 = orientation(p2, q2, q1);

        // General case
        if (o1 != o2 && o3 != o4)
            return true;

        // Special Cases
        // p1, q1 and p2 are collinear and p2 lies on segment p1q1
        if (o1 == 0 && onSegment(p1, p2, q1)) return true;

        // p1, q1 and q2 are collinear and q2 lies on segment p1q1
        if (o2 == 0 && onSegment(p1, q2, q1)) return true;

        // p2, q2 and p1 are collinear and p1 lies on segment p2q2
        if (o3 == 0 && onSegment(p2, p1, q2)) return true;

        // p2, q2 and q1 are collinear and q1 lies on segment p2q2
        if (o4 == 0 && onSegment(p2, q1, q2)) return true;

        return false; // Doesn't fall in any of the above cases
    }

    public Vec2D findIntersectionPoint(Line line) {
        double a1 = this.start.getY() - this.end.getY();
        double b1 = this.end.getX() - this.start.getX();
        double c1 = (this.start.getX() * this.end.getY()) - (this.end.getX() * this.start.getY());

        double a2 = line.start.getY() - line.end.getY();
        double b2 = line.end.getX() - line.start.getX();
        double c2 = (line.start.getX() * line.end.getY()) - (line.end.getX() * line.start.getY());

        return new Vec2D (
                (b1 * c2 - b2 * c1) / (a1 * b2 - a2 * b1),
                (a1 * c2 - a2 * c1) / (a2 * b1 - a1 * b2)
        );
    }

    private boolean onSegment(Vec2D p, Vec2D q, Vec2D r) {
        return (q.getX() <= Math.max(p.getX(), r.getX()) && q.getX() >= Math.min(p.getX(), r.getX()) &&
                q.getY() <= Math.max(p.getY(), r.getY()) && q.getY() >= Math.min(p.getY(), r.getY()));
    }

    // To find orientation of ordered triplet (p, q, r).
    // The function returns following values
    // 0 --> p, q and r are collinear
    // 1 --> Clockwise
    // 2 --> Counterclockwise
    static int orientation(Vec2D p, Vec2D q, Vec2D r) {
        double val = (q.getY() - p.getY()) * (r.getX() - q.getX()) -
                     (q.getX() - p.getX()) * (r.getY() - q.getY());

        if (val == 0) return 0; // collinear

        return val > 0 ? 1 : 2; // clockwise or counterclockwise
    }

    @Override
    public String toString() {
        return "Line{" + "start=" + start + ", end=" + end + " }";
    }
}
