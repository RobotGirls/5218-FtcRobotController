package org.firstinspires.ftc.teamcode.purepursuit;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * BioBuzz (2026-2027) field geometry in Road Runner field coordinates: inches, origin at the
 * center of the field, +x to the right and +y up when looking at biobuzz-field.png (the same
 * orientation MeepMeep uses).
 */
public final class BioBuzzField {
    private BioBuzzField() {}

    public static final double HALF_FIELD = 72;

    /** The central structure covers about the middle 2x2 tiles (measured from the field image). */
    public static final double CENTER_MIN_X = -25.5, CENTER_MAX_X = 25.5;
    public static final double CENTER_MIN_Y = -25.5, CENTER_MAX_Y = 25.5;

    /** Default robot footprint (in) used for collision warnings. */
    public static final double ROBOT_LENGTH = 18, ROBOT_WIDTH = 18;

    /** Corners of a robot footprint, front-left first going counter-clockwise. */
    public static Vector2d[] robotCorners(Pose2d pose, double length, double width) {
        double[][] local = {
                {length / 2, width / 2}, {-length / 2, width / 2},
                {-length / 2, -width / 2}, {length / 2, -width / 2}
        };
        Vector2d[] corners = new Vector2d[4];
        for (int i = 0; i < 4; i++) {
            corners[i] = pose.times(new Vector2d(local[i][0], local[i][1]));
        }
        return corners;
    }

    /** True if the robot footprint overlaps the central structure. */
    public static boolean hitsCenterStructure(Pose2d pose, double length, double width) {
        Vector2d[] robot = robotCorners(pose, length, width);
        Vector2d[] structure = {
                new Vector2d(CENTER_MAX_X, CENTER_MAX_Y), new Vector2d(CENTER_MIN_X, CENTER_MAX_Y),
                new Vector2d(CENTER_MIN_X, CENTER_MIN_Y), new Vector2d(CENTER_MAX_X, CENTER_MIN_Y)
        };
        // separating axis test between two rectangles
        return !separated(robot, structure) && !separated(structure, robot);
    }

    /** True if any part of the robot footprint is outside the field walls. */
    public static boolean outsideField(Pose2d pose, double length, double width) {
        for (Vector2d corner : robotCorners(pose, length, width)) {
            if (Math.abs(corner.x) > HALF_FIELD || Math.abs(corner.y) > HALF_FIELD) {
                return true;
            }
        }
        return false;
    }

    private static boolean separated(Vector2d[] a, Vector2d[] b) {
        for (int i = 0; i < a.length; i++) {
            Vector2d p1 = a[i];
            Vector2d p2 = a[(i + 1) % a.length];
            double nx = p2.y - p1.y;
            double ny = p1.x - p2.x;
            double minA = Double.POSITIVE_INFINITY, maxA = Double.NEGATIVE_INFINITY;
            double minB = Double.POSITIVE_INFINITY, maxB = Double.NEGATIVE_INFINITY;
            for (Vector2d p : a) {
                double d = p.x * nx + p.y * ny;
                minA = Math.min(minA, d);
                maxA = Math.max(maxA, d);
            }
            for (Vector2d p : b) {
                double d = p.x * nx + p.y * ny;
                minB = Math.min(minB, d);
                maxB = Math.max(maxB, d);
            }
            if (maxA < minB || maxB < minA) {
                return true;
            }
        }
        return false;
    }
}
