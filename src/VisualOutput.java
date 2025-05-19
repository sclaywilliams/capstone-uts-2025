import javax.swing.*;
import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.Rectangle2D;
import java.awt.image.ColorModel;
import java.util.ArrayList;

class VisualOutput extends JComponent {

    // instance variables //

    private final World world;
    private ArrayList<Robot> robots;
    private ArrayList<Obstacle> obstacles;

    // constructors //

    public VisualOutput() {
        this.world = new World();
    }

    public VisualOutput(World world) {
        this.world = world;
        this.robots = world.getRobots();
        this.obstacles = world.getObstacles();
    }

    // main drawing function //

    public void paintComponent(Graphics g) {
        Graphics2D g2d = (Graphics2D) g;

        boolean customRobotColor = false;
        boolean customSpringColor = false;

        boolean showLabel = false;
        boolean showHeatmap = true;


        // draw world boundary //
        //                WorldBoundary worldBoundary = world.getWorldBoundary();
        //        int minX = worldBoundary.getMinX();
        //        int minY = worldBoundary.getMinY();
        //        int width = worldBoundary.getWidth();
        //        int height = worldBoundary.getHeight();
        //        g2d.drawRect(minX, minY, width, height);

        // draw heatmap //
        if (showHeatmap) {
            drawHeatmap(world, g2d);
        }

        // draw obstacles //
        if (!obstacles.isEmpty()) {
            for (Obstacle obstacle : obstacles) {
                drawObstacle(g2d, obstacle);
            }
        }

        // draw robots and springs //
        int robotSize = 10;
        if (!robots.isEmpty()) {
            for (Robot robot : robots) {

                // draw robot //
                Color robotColor = customRobotColor ? getRobotColor(robot, g2d) : Color.RED;
                g2d.setColor(robotColor);
                g2d.drawOval((int) robot.getPosX() - robotSize / 2, (int) robot.getPosY() - robotSize / 2, robotSize, robotSize);
                g2d.fillOval((int) robot.getPosX() - robotSize / 2, (int) robot.getPosY() - robotSize / 2, robotSize, robotSize);

                boolean label = false;
                if (showLabel) {
                    g2d.setColor(Color.BLACK);
                    g2d.drawString(String.valueOf(robot.getId()), (int) robot.getPosX(), (int) robot.getPosY());
                }
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
                    Color lineColor = customSpringColor ? getSpringColor(distance, spring) : Color.BLACK;
                    g2d.setColor(lineColor);
                    g2d.drawLine((int) springRobots[0].getPosX(), (int) springRobots[0].getPosY(), (int) springRobots[1].getPosX(), (int) springRobots[1].getPosY());

                }
            }
        }
    }

    // helpers //

    private Color getGradient(Color c1, Color c2, double ratio) {
        double r = c1.getRed() * ratio + c2.getRed() * (1 - ratio);
        double g = c1.getGreen() * ratio + c2.getGreen() * (1 - ratio);
        double b = c1.getBlue() * ratio + c2.getBlue() * (1 - ratio);
        return new Color((int) r, (int) g, (int) b);
    }

    private Color getThreeColorGradient(Color c1, Color c2, Color c3, double ratio) {
        if (ratio < 0.5) {
            return getGradient(c2, c1, ratio * 2);
        } else {
            return getGradient(c3, c2, (ratio - 0.5) * 2);
        }
    }

    private Color getLineColor(double distance, double maxDistance) {
        double normalisedDistance = Utils.clampDouble(distance / maxDistance, 0.0, 1.0);
        if (normalisedDistance > 0.7) {
            return new Color(255, (int) (255 * normalisedDistance), 0);
        } else {
            return new Color((int) (255 * normalisedDistance), 255, 0);
        }
    }

    private Color getHeatmapColor(double distance, double maxDistance) {
        double normalisedDistance = Utils.clampDouble(distance / maxDistance, 0.0, 1.0);
        Color closestColor = Color.GREEN;
        Color midColor = Color.YELLOW;
        Color furthestColor = Color.RED;
//        return getGradient(furthestColor, closestColor, normalisedDistance);
        return getThreeColorGradient(closestColor, midColor, furthestColor, normalisedDistance);
    }

    private Color getSignalHeatmapColor(double signalStrength, double maximumSignalStrength, double minimumSignalStrength) {
        double signalPercent = (signalStrength - minimumSignalStrength) / (maximumSignalStrength - minimumSignalStrength);
        double normalisedSignalStrength = 1 - Utils.clampDouble(signalPercent, 0.0, 1.0);
        Color closestColor = Color.GREEN;
        Color midColor = Color.YELLOW;
        Color furthestColor = Color.RED;
        return getThreeColorGradient(closestColor, midColor, furthestColor, normalisedSignalStrength);
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

    private Color getRobotColor(Robot robot, Graphics2D g2d) {
//        return new Color(255, 0, 0);
//        Obstacle checkedArea = robot.calculateEdge();
//        if (checkedArea != null) {
//            drawObstacle(g2d, checkedArea);
//        }

//        double theta = Math.toRadians(0);
//        double x = robot.getPosX();
//        double y = robot.getPosY();
////        Vec2D mV = new Vec2D(
////                x * Math.cos(theta) - y * Math.sin(theta),
////                x * Math.sin(theta) + y * Math.cos(theta)
////        );
//        Vec2D mV = new Vec2D(
//                Math.cos(theta),
//                Math.sin(theta)
//        );
//
//        Vec2D rayCastPoint = Vec2D.add(robot.getPosition(), Vec2D.multiplyMagnitude(mV, 1000));

//        g2d.setColor(Color.BLACK);
//        g2d.drawLine((int) robot.getPosX(), (int) robot.getPosY(), (int) rayCastPoint.getX(), (int) rayCastPoint.getY());

        if (robot.calculateEdge()[0] != -1) {
            return new Color(0, 0, 255);
        }
        return new Color(255, 0, 0);
    }

    private void drawObstacle(Graphics2D g2d, Obstacle obstacle) {
        Color obstacleFillColor = Color.WHITE;
        if (obstacle.isEnclosed()) {
            g2d.setColor(obstacleFillColor);
            g2d.fillPolygon(obstacle.getXPoints(), obstacle.getYPoints(), obstacle.getNumVertices());
        }
        int obstacleWallThickness = obstacle.isEnclosed() ? 2 : 5;
        g2d.setStroke(new BasicStroke(obstacleWallThickness));
        Color obstacleWallColor = Color.BLACK;
        g2d.setColor(obstacleWallColor);
        for (Line line : obstacle.getEdges()) {
            g2d.drawLine(
                    (int) line.getStart().getX(),
                    (int) line.getStart().getY(),
                    (int) line.getEnd().getX(),
                    (int) line.getEnd().getY()
                        );
        }
        g2d.setStroke(new BasicStroke());
    }

    private void drawHeatmap(World world, Graphics2D g2d) {
        for (int i = world.margin; i < world.margin + world.width; i++) {
            for (int j = world.margin; j < world.margin + world.height; j++) {
                Vec2D pixelPosition = new Vec2D(i, j);
                double minDistance = Double.MAX_VALUE;

                // get distance to the closest robot //
                for (Robot robot : robots) {
                    double distance = Utils.getDistanceBetweenPoints(robot.getPosition(), pixelPosition);
                    if (distance < minDistance) {
                        minDistance = distance;
                    }
                }
                double signalStrength = world.calculateMaximumSignalStrength(pixelPosition);

                // colour pixel according to distance //
//                Color pixelColor = getHeatmapColor(minDistance, 100);
                Color pixelColor = getSignalHeatmapColor(signalStrength, 120, 70);
                g2d.setColor(pixelColor);
                g2d.drawOval(i, j, 1, 1);
            }
        }
    }

}
