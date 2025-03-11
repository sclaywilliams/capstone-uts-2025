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
                g2d.drawOval((int) robot.pos_x - robotSize/2, (int) robot.pos_y - robotSize/2, robotSize, robotSize);
                g2d.fillOval((int) robot.pos_x - robotSize/2, (int) robot.pos_y - robotSize/2, robotSize, robotSize);

                // draw communication lines
                g2d.setColor(Color.green);
                ArrayList<Robot> localRobots = world.getLocalRobots(robot, robots);
                System.out.println("Robot " + robot.id + " local robots: " + localRobots.toString());
                for (Robot localRobot : localRobots) {
                    if (robot != localRobot) {
                        g2d.drawLine((int) robot.pos_x,(int) robot.pos_y, (int) localRobot.pos_x, (int) localRobot.pos_y);
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
        double maxSeparation = 50;

        ArrayList<Robot> robots = world.robots;

        // move robots //
        for (int i = 0; i < 1000; i++) {
            for (Robot robot : robots) {
                double[] vectorSum = world.getWeightedLocalVectorSum(robot, robots, maxSeparation);
                double vectorMagnitude = Math.sqrt(Math.pow(vectorSum[0], 2) + Math.pow(vectorSum[1], 2));

                // robot move calc
                if (vectorMagnitude > 0) {
                    robot.pos_x += (maxSeparation / vectorMagnitude) * vectorSum[0];
                    robot.pos_y += (maxSeparation / vectorMagnitude) * vectorSum[1];
                }

                // boundary clamping
                if (robot.pos_x < world.worldBoundary.min_x) {
                    robot.pos_x = world.worldBoundary.min_x;
                }
                if (robot.pos_y < world.worldBoundary.min_y) {
                    robot.pos_y = world.worldBoundary.min_y;
                }
                if (robot.pos_x > world.worldBoundary.max_x) {
                    robot.pos_x = world.worldBoundary.max_x;
                }
                if (robot.pos_y > world.worldBoundary.max_y) {
                    robot.pos_y = world.worldBoundary.max_y;
                }

//                System.out.println(robot.id + " " + robot.pos_x);
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