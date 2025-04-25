package org.firstinspires.ftc.teamcode.archive;

import java.util.ArrayList;

public class Util {
    public static class Coordinate {
        double x, y;

        public Coordinate(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public double distanceTo(Coordinate other) {
            return Math.sqrt(Math.pow(other.x - x, 2) + Math.pow(other.y - y, 2));
        }

        public double slopeWith(Coordinate other) {
            return (other.y - y) / (other.x - x);
        }
    }

    public static double max(ArrayList<Double> collection) {
        double k = 0;
        for (Double x : collection) if (x > k) k = x;
        return k;
    }
}
