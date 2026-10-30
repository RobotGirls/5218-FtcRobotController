package org.firstinspires.ftc.teamcode.purepursuit;

import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;
import java.util.List;

/** Geometry helpers for pure pursuit (Gluten Free's MyMath / MyPosition helpers). */
public final class PurePursuitMath {
    private PurePursuitMath() {}

    /** Wraps an angle into [-PI, PI). */
    public static double angleWrap(double radians) {
        double wrapped = (radians + Math.PI) % (2 * Math.PI);
        if (wrapped < 0) {
            wrapped += 2 * Math.PI;
        }
        return wrapped - Math.PI;
    }

    public static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Where the segment p1 -> p2 crosses a circle, as fractions t in [0, 1] along the segment
     * (smallest first). Uses the vector form of the quadratic, so unlike the slope based version
     * Gluten Free used, vertical and horizontal segments need no special casing.
     */
    public static double[] segmentCircleIntersections(Vector2d center, double radius,
                                                      Vector2d p1, Vector2d p2) {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double fx = p1.x - center.x;
        double fy = p1.y - center.y;

        double a = dx * dx + dy * dy;
        if (a < 1e-12) {
            return new double[0];
        }
        double b = 2 * (fx * dx + fy * dy);
        double c = fx * fx + fy * fy - radius * radius;
        double discriminant = b * b - 4 * a * c;
        if (discriminant < 0) {
            return new double[0];
        }

        double root = Math.sqrt(discriminant);
        double t1 = (-b - root) / (2 * a);
        double t2 = (-b + root) / (2 * a);

        boolean t1In = t1 >= 0 && t1 <= 1;
        boolean t2In = t2 >= 0 && t2 <= 1 && t2 - t1 > 1e-12;
        if (t1In && t2In) {
            return new double[] {t1, t2};
        } else if (t1In) {
            return new double[] {t1};
        } else if (t2In) {
            return new double[] {t2};
        }
        return new double[0];
    }

    /** Points where the segment p1 -> p2 crosses the circle. */
    public static List<Vector2d> lineCircleIntersection(Vector2d center, double radius,
                                                        Vector2d p1, Vector2d p2) {
        List<Vector2d> result = new ArrayList<>();
        for (double t : segmentCircleIntersections(center, radius, p1, p2)) {
            result.add(lerp(p1, p2, t));
        }
        return result;
    }

    /** Fraction t in [0, 1] of the point on segment p1 -> p2 closest to p. */
    public static double closestT(Vector2d p, Vector2d p1, Vector2d p2) {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double lengthSq = dx * dx + dy * dy;
        if (lengthSq < 1e-12) {
            return 0;
        }
        return clip(((p.x - p1.x) * dx + (p.y - p1.y) * dy) / lengthSq, 0, 1);
    }

    /** Gluten Free's clipToLine: the point on segment p1 -> p2 closest to p. */
    public static Vector2d clipToSegment(Vector2d p, Vector2d p1, Vector2d p2) {
        return lerp(p1, p2, closestT(p, p1, p2));
    }

    public static Vector2d lerp(Vector2d a, Vector2d b, double t) {
        return new Vector2d(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
    }

    public static double distance(Vector2d a, Vector2d b) {
        return Math.hypot(a.x - b.x, a.y - b.y);
    }
}
