public class Utils {

    public static int[] intVectorSum(int[] a, int[] b) {
        if (a.length != b.length) {
            return null;
        }
        int[] c = new int[a.length];
        for (int i = 0; i < a.length; i++) {
            c[i] = a[i] + b[i];
        }
        return c;
    }

    public static double[] doubleVectorSum(double[] a, double[] b) {
        if (a.length != b.length) {
            return null;
        }
        double[] c = new double[a.length];
        for (int i = 0; i < a.length; i++) {
            c[i] = a[i] + b[i];
        }
        return c;
    }

    public static int clampInt(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }

    public static double clampDouble(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public static boolean checkLineIntersection(Line l1, Line l2) {
        if (l1.getSlope() == l2.getSlope()) {
            return false;
        }

        // check if intersects segment //

        return false;
    }

    public static double getDistanceBetweenPoints(Vec2D point1, Vec2D point2) {
        return Math.sqrt(Math.pow(point1.getX() - point2.getX(), 2) + Math.pow(point1.getY() - point2.getY(), 2));
    }

    public static double calculateFSPL(double distance, double frequency, double totalGain) {
        return 20 * Math.log10(distance) + 20 * Math.log10(frequency) + 20 * Math.log10(4 * Math.PI / Variables.LIGHT_SPEED) + totalGain;
    }

    public static double getAttenuation(String material) {
        return switch (material) {
            case "wood" -> 6;
            case "drywall" -> 5;
            case "concrete" -> 20;
            case "singleBrick" -> 10;
            case "doubleBrick" -> 28;
            case "glass" -> 4;
            case "tintedGlass" -> 14;
            default -> 0;
        };
    }

}
