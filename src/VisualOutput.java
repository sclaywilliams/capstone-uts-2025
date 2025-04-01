import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;

class VisualOutput extends JComponent {

    // instance variables //

    private final World world;
    private ArrayList<Robot> robots = new ArrayList<Robot>();

    // constructors //

    public VisualOutput() {
        this.world = new World();
    }

    public VisualOutput(World world) {
        this.world = world;
        this.robots = world.getRobots();
    }

    // main drawing function //

    public void paintComponent(Graphics g) {
        Graphics2D g2d = (Graphics2D) g;

        // draw world boundary //
        WorldBoundary worldBoundary = world.getWorldBoundary();
        int minX = worldBoundary.getMinX();
        int minY = worldBoundary.getMinY();
        int width = worldBoundary.getWidth();
        int height = worldBoundary.getHeight();
        g2d.drawRect(minX, minY, width, height);

        int robotSize = 5;
        if (!robots.isEmpty()) {
            for (Robot robot : robots) {

                // draw robot //
                g2d.setColor(Color.red);
                g2d.drawOval((int) robot.getPosX() - robotSize/2, (int) robot.getPosY() - robotSize/2, robotSize, robotSize);
                g2d.fillOval((int) robot.getPosX() - robotSize/2, (int) robot.getPosY() - robotSize/2, robotSize, robotSize);

                // draw connection lines //
//                ArrayList<Robot> localRobots = world.getLocalRobots(robot, robots);
//                for (Robot localRobot : localRobots) {
//                    if (robot != localRobot) {
//                        int distance = (int) world.getRobotSeparationDistance(robot, localRobot);
//                        Color lineColor = getLineColor(distance, robot.getCommunicationDistance());
//                        g2d.setColor(lineColor);
//                        g2d.drawLine((int) robot.getPosX(),(int) robot.getPosY(), (int) localRobot.getPosX(), (int) localRobot.getPosY());
//                    }
//                }

                // draw springs //
                ArrayList<Spring> springs = robot.getSprings();
                if (springs == null || springs.isEmpty()) {
                    continue;
                }
                for (Spring spring : springs) {
                    if (spring == null) {
                        continue;
                    }
                    Robot[] springRobots = spring.getRobots();
                    int distance = (int) world.getRobotSeparationDistance(springRobots[0], springRobots[1]);
//                    Color lineColor = getLineColor(distance, spring.getNatLength() * 2);
                    Color lineColor = getSpringColor(distance, spring);
                    g2d.setColor(lineColor);
                    g2d.drawLine((int) springRobots[0].getPosX(),(int) springRobots[0].getPosY(), (int) springRobots[1].getPosX(), (int) springRobots[1].getPosY());

                }
            }
        }
    }

    // helpers //

    private Color getLineColor(double distance, double maxDistance) {
        double normalisedDistance = Utils.clampDouble(distance / maxDistance, 0.0, 1.0);
        if (normalisedDistance > 0.7) {
            return new Color(255, (int) (255 * normalisedDistance), 0);
        }
        else {
            return new Color((int) (255 * normalisedDistance), 255, 0);
        }
    }

    private Color getSpringColor(int distance, Spring spring) {
        double natLength = spring.getNatLength();
        double ratio = distance / natLength;

        if (distance <= natLength) {
            return new Color((int) (255 - (255 * ratio)), (int) (255 * ratio), 0);
        } else {
            if (ratio > 2) {
                return new Color(255, 0, 0);
            } else {
                return new Color((int) (255 * (ratio - 1)), (int) (255 - (255 * (ratio - 1))), 0);
            }
        }

    }

}
