public class WorldBoundary {

    public int min_x;
    public int min_y;
    public int max_x;
    public int max_y;

    public WorldBoundary(int min_x, int min_y, int max_x, int max_y) {
        this.min_x = min_x;
        this.min_y = min_y;
        this.max_x = max_x;
        this.max_y = max_y;
    }

    public WorldBoundary() {
        this.min_x = 50;
        this.min_y = 50;
        this.max_x = 550;
        this.max_y = 550;
    }

}
