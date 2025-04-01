public class WorldBoundary {

    private final int minX;
    private final int minY;
    private final int maxX;
    private final int maxY;

    public WorldBoundary(int minX, int minY, int maxX, int maxY) {
        this.minX = minX;
        this.minY = minY;
        this.maxX = maxX;
        this.maxY = maxY;
    }

    public WorldBoundary() {
        this.minX = 50;
        this.minY = 50;
        this.maxX = 850;
        this.maxY = 850;
    }

    // getters //

    public int getMinX() {
        return minX;
    }

    public int getMinY() {
        return minY;
    }

    public int getMaxX() {
        return maxX;
    }

    public int getMaxY() {
        return maxY;
    }

    // helpers //

    public int getWidth() {
        return maxX - minX;
    }

    public int getHeight() {
        return maxY - minY;
    }

}
