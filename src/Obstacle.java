import java.util.ArrayList;

public class Obstacle {

    // instance variables //

    private final ArrayList<Vec2D> points;
    private final ArrayList<Line> edges;
    private final boolean enclosed;
    private final String material;

    // constructors //

    public Obstacle(ArrayList<Vec2D> points) {
        this.points = points;
        edges = buildEdges(points, false);
        enclosed = false;
        material = "concrete"; // default value //
    }

    // basic polygon constructor //
    public Obstacle(Vec2D position, int width, int height, String shape, boolean leaveOpen) {
        ArrayList<Vec2D> points = getShapeVertices(position, width, height, shape);
        this.points = points;
        edges = buildEdges(points, leaveOpen);
        enclosed = !leaveOpen;
        material = "concrete";
    }

    public Obstacle(ArrayList<Vec2D> points, String material) {
        this.points = points;
        edges = buildEdges(points, false);
        enclosed = false;
        this.material = material;
    }

    // basic polygon constructor //
    public Obstacle(Vec2D position, int width, int height, String shape, boolean leaveOpen, String material) {
        ArrayList<Vec2D> points = getShapeVertices(position, width, height, shape);
        this.points = points;
        edges = buildEdges(points, leaveOpen);
        enclosed = !leaveOpen;
        this.material = material;
    }

    private ArrayList<Vec2D> getShapeVertices(Vec2D position, double width, double height, String shape) {
        ArrayList<Vec2D> vertices = new ArrayList<>();
        double x = position.getX();
        double y = position.getY();

        switch (shape) {
            case "rectangle":
                vertices.add(position);
                vertices.add(new Vec2D(position.getX() + width, position.getY()));
                vertices.add(new Vec2D(position.getX() + width, position.getY() + height));
                vertices.add(new Vec2D(position.getX(), position.getY() + height));
                break;
            case "triangle":
                vertices.add(new Vec2D(x - width/2, y - height/2));
                vertices.add(new Vec2D(x, y + height/2));
                vertices.add(new Vec2D(x + width/2, y - height/2));
                break;
            case "star":
                vertices.add(new Vec2D(x - width/2, y - height/4));
                vertices.add(new Vec2D(x - width/3, y));
                vertices.add(new Vec2D(x - width/2, y + height/4));
                vertices.add(new Vec2D(x - width/6, y + height/4));
                vertices.add(new Vec2D(x, y + height/2));
                vertices.add(new Vec2D(x + width/6, y + height/4));
                vertices.add(new Vec2D(x + width/2, y + height/4));
                vertices.add(new Vec2D(x + width/3, y));
                vertices.add(new Vec2D(x + width/2, y - height/4));
                vertices.add(new Vec2D(x + width/6, y - height/4));
                vertices.add(new Vec2D(x, y - height/2));
                vertices.add(new Vec2D(x - width/6, y - height/4));
                break;
            case "office1":
                vertices.add(new Vec2D(x - width/2, y + height/2));
                vertices.add(new Vec2D(x - width/2, y - height/2));
                vertices.add(new Vec2D(x + width/2, y - height/2));
                vertices.add(new Vec2D(x + width/2, y + height/2));
                vertices.add(new Vec2D(x, y + height/2));
                break;
            case "office2":
                vertices.add(new Vec2D(x - width/4, y + height/2));
                vertices.add(new Vec2D(x - width/2, y + height/2));
                vertices.add(new Vec2D(x - width/2, y - height/2));
                vertices.add(new Vec2D(x + width/2, y - height/2));
                vertices.add(new Vec2D(x + width/2, y + height/2));
                vertices.add(new Vec2D(x + width/4, y + height/2));
                break;
        }
        return vertices;
    }

    /** buildEdges
     *
     * @param points list of points (in clockwise order)
     * @return list of edges joining provided points
     */
    private ArrayList<Line> buildEdges(ArrayList<Vec2D> points, boolean leaveOpen) {
        ArrayList<Line> edges = new ArrayList<>();
        for (int i = 0; i < points.size() - 1; i++) {
            Vec2D p1 = points.get(i);
            Vec2D p2 = points.get(i + 1);
            edges.add(new Line(p1, p2));
        }
        if (!leaveOpen) {
            edges.add(new Line(points.getFirst(), points.getLast()));
        }
        return edges;
    }

    // getters //

    public ArrayList<Vec2D> getPoints() {
        return points;
    }

    public ArrayList<Line> getEdges() {
        return edges;
    }

    public boolean isEnclosed() {
        return enclosed;
    }

    public String getMaterial() {
        return material;
    }

    public int getNumVertices() {
        return points.size();
    }

    public int[] getXPoints() {
        int[] xPoints = new int[points.size()];
        for (int i = 0; i < points.size(); i++) {
            xPoints[i] = (int) points.get(i).getX();
        }
        return xPoints;
    }

    public int[] getYPoints() {
        int[] yPoints = new int[points.size()];
        for (int i = 0; i < points.size(); i++) {
            yPoints[i] = (int) points.get(i).getY();
        }
        return yPoints;
    }

    // helpers //

    public boolean contains(Vec2D point) {
        Line line = new Line(point, new Vec2D(Double.MAX_VALUE, point.getY()));
        int intersects = 0;
        for (Line edge : edges) {
            if (edge.intersects(line)) intersects++;
        }
        return intersects % 2 == 1;
    }

    public boolean intersects(Line line) {
        for (Line edge : edges) {
            if (edge.intersects(line)) return true;
        }
        return false;
    }

}
