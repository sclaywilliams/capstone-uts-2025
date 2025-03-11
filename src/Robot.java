public class Robot {

    public int id;
    double pos_x;
    double pos_y;
    double communicationDistance;

    public Robot(int id, double pos_x, double pos_y) {
        this.id = id;
        this.pos_x = pos_x;
        this.pos_y = pos_y;
        this.communicationDistance = 50;
    }
}
