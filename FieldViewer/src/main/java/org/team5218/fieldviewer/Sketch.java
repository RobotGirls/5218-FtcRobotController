package org.team5218.fieldviewer;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.purepursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.purepursuit.Route;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * A path built in the FieldViewer's path editor. The first point is the start, the rest are
 * waypoints. It can be simulated like any other route and copied out as Java to paste into
 * purepursuit/paths.
 */
final class Sketch {
    /** One waypoint; speed / lookahead override the path's defaults for the segment ending here. */
    static final class Point {
        final double x, y;
        final Double speed, lookahead;

        Point(double x, double y, Double speed, Double lookahead) {
            this.x = x;
            this.y = y;
            this.speed = speed;
            this.lookahead = lookahead;
        }
    }

    final List<Point> points = new ArrayList<>();
    String name = "";
    boolean reversed;
    double moveSpeed = 0.6;
    double followDistance = 16;
    /** Field headings in radians; null means "not set". */
    Double startHeading, endHeading, fixedHeading;

    void add(double x, double y, Double speed, Double lookahead) {
        points.add(new Point(x, y, speed, lookahead));
    }

    boolean isUsable() {
        return points.size() >= 2;
    }

    /** The chosen start heading, or facing the first waypoint (away from it when reversed). */
    double startHeading() {
        if (startHeading != null) {
            return startHeading;
        }
        Point a = points.get(0), b = points.get(1);
        double heading = Math.atan2(b.y - a.y, b.x - a.x) + (reversed ? Math.PI : 0);
        return Math.atan2(Math.sin(heading), Math.cos(heading));
    }

    private String displayName() {
        return name == null || name.trim().isEmpty() ? "My Route" : name.trim();
    }

    Route toRoute() {
        Point start = points.get(0);
        PurePursuitPath.Builder path = PurePursuitPath.builder().name(displayName());
        for (int i = 1; i < points.size(); i++) {
            Point p = points.get(i);
            path.moveSpeed(p.speed != null ? p.speed : moveSpeed)
                    .followDistance(p.lookahead != null ? p.lookahead : followDistance)
                    .to(p.x, p.y);
        }
        if (reversed) {
            path.reversed();
        }
        if (endHeading != null) {
            path.endHeading(endHeading);
        }
        if (fixedHeading != null) {
            path.fixedHeading(fixedHeading);
        }
        return Route.builder("Sketch", Route.Alliance.NONE, new Pose2d(start.x, start.y, startHeading()))
                .description("Built in the FieldViewer")
                .follow(path.build())
                .build();
    }

    /** Java to paste into BioBuzzRoutes / TestRoutes. */
    String toJava() {
        Point start = points.get(0);
        String constant = displayName().toUpperCase(Locale.US).replaceAll("[^A-Z0-9]+", "_").replaceAll("^_|_$", "");
        if (constant.isEmpty() || Character.isDigit(constant.charAt(0))) {
            constant = "ROUTE_" + constant;
        }

        StringBuilder sb = new StringBuilder();
        sb.append("public static final Route ").append(constant).append(" = Route.builder(\n")
                .append("                \"").append(displayName().replace("\"", "'")).append("\", Alliance.NONE, new Pose2d(")
                .append(num(start.x)).append(", ").append(num(start.y)).append(", Math.toRadians(")
                .append(num(Math.toDegrees(startHeading()))).append(")))\n")
                .append("        .follow(PurePursuitPath.builder().name(\"")
                .append(displayName().toLowerCase(Locale.US).replace("\"", "'")).append("\")\n");

        // only write a setting when it changes, like you would by hand
        double speed = 0.6, lookahead = 16;
        for (int i = 1; i < points.size(); i++) {
            Point p = points.get(i);
            double wantSpeed = p.speed != null ? p.speed : moveSpeed;
            double wantLookahead = p.lookahead != null ? p.lookahead : followDistance;
            if (Math.abs(wantSpeed - speed) > 1e-9) {
                sb.append("                .moveSpeed(").append(num(wantSpeed)).append(")\n");
                speed = wantSpeed;
            }
            if (Math.abs(wantLookahead - lookahead) > 1e-9) {
                sb.append("                .followDistance(").append(num(wantLookahead)).append(")\n");
                lookahead = wantLookahead;
            }
            sb.append("                .to(").append(num(p.x)).append(", ").append(num(p.y)).append(")\n");
        }
        if (reversed) {
            sb.append("                .reversed()\n");
        }
        if (fixedHeading != null) {
            sb.append("                .fixedHeading(Math.toRadians(").append(num(Math.toDegrees(fixedHeading))).append("))\n");
        }
        if (endHeading != null) {
            sb.append("                .endHeading(Math.toRadians(").append(num(Math.toDegrees(endHeading))).append("))\n");
        }
        sb.append("                .build())\n")
                .append("        .build();\n");
        return sb.toString();
    }

    /** Up to two decimals, no trailing zeros: 90, 0.65, 12.5 (89.99999 from degree math prints 90). */
    private static String num(double value) {
        double rounded = Math.round(value * 100) / 100.0;
        if (rounded == Math.rint(rounded)) {
            return String.valueOf((long) rounded);
        }
        return String.format(Locale.US, "%.2f", rounded).replaceAll("0+$", "");
    }
}
