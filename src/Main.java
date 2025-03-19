import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;
import javax.swing.JComponent;


class ShapeDrawingComponent extends JComponent {

    public World world;
    public ArrayList<Robot> robots = new ArrayList<Robot>();

    public ShapeDrawingComponent() {
        this.world = new World();
    }

    public ShapeDrawingComponent(World world) {
        this.world = world;
        this.robots = world.robots;
    }

    public void paintComponent(Graphics g) {
        Graphics2D g2d = (Graphics2D) g;
        g2d.drawRect(50, 50, 500, 500);
//        g2d.drawOval(50, 50, 10, 10);
        int robotSize = 5;
        if (!robots.isEmpty()) {
            for (Robot robot : robots) {
                // draw robot
                g2d.setColor(Color.red);
                g2d.drawOval((int) robot.posX - robotSize/2, (int) robot.posY - robotSize/2, robotSize, robotSize);
                g2d.fillOval((int) robot.posX - robotSize/2, (int) robot.posY - robotSize/2, robotSize, robotSize);

                // draw communication lines
//                g2d.setColor(Color.green);
                ArrayList<Robot> localRobots = world.getLocalRobots(robot, robots);
//                System.out.println("Robot " + robot.id + " local robots: " + localRobots.toString());
                for (Robot localRobot : localRobots) {
                    if (robot != localRobot) {
                        int distance = (int) world.getRobotSeparationDistance(robot, localRobot);
//                        System.out.println("Distance: " + distance);
                        double distanceColor = Utils.clampDouble(((distance) / (robot.communicationDistance)) * 255, 0, 255);
//                        System.out.println("DistanceColor: " + distanceColor);
                        Color lineColor = new Color(255 - (int) distanceColor, (int) distanceColor, 0);
//                        Color lineColor = new Color((int) distanceColor,255 - (int) distanceColor, 0);
                        g2d.setColor(lineColor);
                        g2d.drawLine((int) robot.posX,(int) robot.posY, (int) localRobot.posX, (int) localRobot.posY);
                    }

                }
            }
        }
    }

}

public class Main {

    public static void main(String[] args) {
        World world = new World();
        world.createRobots(14, 14);

        // TODO: remove double definition (use robot intrinsic)
        double communicationDistance = 50;

//        System.out.println();

        ArrayList<Robot> robots = world.robots;

        // move robots //
        for (int i = 0; i < 1000; i++) {
            for (Robot robot : robots) {
                double[] vectorSum = world.getWeightedLocalVectorSum(robot, robots, communicationDistance);
                double vectorMagnitude = Math.sqrt(Math.pow(vectorSum[0], 2) + Math.pow(vectorSum[1], 2));

                // robot move calc
                if (vectorMagnitude > 0) {
                    robot.posX += (communicationDistance / vectorMagnitude) * vectorSum[0];
                    robot.posY += (communicationDistance / vectorMagnitude) * vectorSum[1];
                }

                // boundary clamping
                if (robot.posX < world.worldBoundary.min_x) {
                    robot.posX = world.worldBoundary.min_x;
                }
                if (robot.posY < world.worldBoundary.min_y) {
                    robot.posY = world.worldBoundary.min_y;
                }
                if (robot.posX > world.worldBoundary.max_x) {
                    robot.posX = world.worldBoundary.max_x;
                }
                if (robot.posY > world.worldBoundary.max_y) {
                    robot.posY = world.worldBoundary.max_y;
                }

//                System.out.println(robot.id + " " + robot.posX);
            }
        }


        // draw robots //

        JFrame frame = new JFrame();
        frame.setSize(600, 630);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        frame.getContentPane().add(new ShapeDrawingComponent(world));

        frame.setVisible(true);




    }




}