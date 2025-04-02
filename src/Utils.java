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
}
