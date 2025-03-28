public class WorldBoundary {

    public int minX;
    public int minY;
    public int maxX;
    public int maxY;

    public WorldBoundary(int minX, int minY, int maxX, int maxY) {
        this.minX = minX;
        this.minY = minY;
        this.maxX = maxX;
        this.maxY = maxY;
    }

    public WorldBoundary() {
        this.minX = 50;
        this.minY = 50;
        this.maxX = 550;
        this.maxY = 550;
    }

}
