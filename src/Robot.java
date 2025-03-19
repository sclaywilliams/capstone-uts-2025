public class Robot {

    public int id;
    double posX;
    double posY;
    double communicationDistance;

    public Robot(int id, double posX, double posY) {
        this.id = id;
        this.posX = posX;
        this.posY = posY;
        this.communicationDistance = 100;
    }
}
