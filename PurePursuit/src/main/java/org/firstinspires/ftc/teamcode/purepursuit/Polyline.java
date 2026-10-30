package org.firstinspires.ftc.teamcode.purepursuit;

import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Connected line segments with the distance along the path ("arc length") of every vertex.
 * Tracking arc length lets the follower only ever move forwards along the path, which keeps it
 * from skipping ahead on paths that loop back on themselves (out and back, hairpins).
 */
final class Polyline {
    final List<Vector2d> vertices;
    /** cumulative[i] = distance along the path from vertex 0 to vertex i */
    private final double[] cumulative;

    Polyline(List<Vector2d> vertices) {
        if (vertices.size() < 2) {
            throw new IllegalArgumentException("a polyline needs at least two vertices");
        }
        this.vertices = Collections.unmodifiableList(new ArrayList<>(vertices));
        cumulative = new double[vertices.size()];
        for (int i = 1; i < vertices.size(); i++) {
            cumulative[i] = cumulative[i - 1]
                    + PurePursuitMath.distance(vertices.get(i - 1), vertices.get(i));
        }
    }

    double length() {
        return cumulative[cumulative.length - 1];
    }

    int segmentCount() {
        return vertices.size() - 1;
    }

    double segmentStart(int segment) {
        return cumulative[segment];
    }

    double segmentLength(int segment) {
        return cumulative[segment + 1] - cumulative[segment];
    }

    /** Index of the segment containing the given distance along the path. */
    int segmentAt(double s) {
        for (int i = 0; i < segmentCount(); i++) {
            if (s < cumulative[i + 1]) {
                return i;
            }
        }
        return segmentCount() - 1;
    }

    Vector2d pointAt(double s) {
        s = PurePursuitMath.clip(s, 0, length());
        int segment = segmentAt(s);
        double length = segmentLength(segment);
        double t = length < 1e-12 ? 0 : (s - cumulative[segment]) / length;
        return PurePursuitMath.lerp(vertices.get(segment), vertices.get(segment + 1), t);
    }

    /**
     * Distance along the path of the point closest to p, only looking between fromS and toS.
     * Earlier points win ties so the robot doesn't jump ahead where the path doubles back.
     */
    double closestS(Vector2d p, double fromS, double toS) {
        double bestS = fromS;
        double bestDistance = Double.POSITIVE_INFINITY;
        for (int i = 0; i < segmentCount(); i++) {
            double start = cumulative[i];
            double length = segmentLength(i);
            if (start + length < fromS || start > toS) {
                continue;
            }
            double t;
            if (length < 1e-12) {
                t = 0;
            } else {
                t = PurePursuitMath.closestT(p, vertices.get(i), vertices.get(i + 1));
                double minT = Math.max(0, (fromS - start) / length);
                double maxT = Math.min(1, (toS - start) / length);
                t = PurePursuitMath.clip(t, minT, maxT);
            }
            double s = start + t * length;
            double d = PurePursuitMath.distance(p, pointAt(s));
            if (d < bestDistance - 1e-9) {
                bestDistance = d;
                bestS = s;
            }
        }
        return bestS;
    }

    /**
     * Distance along the path of the furthest point between fromS and toS that is exactly
     * radius away from center (Gluten Free's getFollowPointPath), or NaN if there isn't one.
     */
    double furthestIntersectionS(Vector2d center, double radius, double fromS, double toS) {
        double best = Double.NaN;
        for (int i = 0; i < segmentCount(); i++) {
            double start = cumulative[i];
            double length = segmentLength(i);
            if (start + length < fromS || start > toS) {
                continue;
            }
            for (double t : PurePursuitMath.segmentCircleIntersections(
                    center, radius, vertices.get(i), vertices.get(i + 1))) {
                double s = start + t * length;
                if (s >= fromS - 1e-9 && s <= toS + 1e-9 && (Double.isNaN(best) || s > best)) {
                    best = s;
                }
            }
        }
        return best;
    }
}
