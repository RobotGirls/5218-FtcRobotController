package org.firstinspires.ftc.teamcode.purepursuit;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * One snapshot of what the robot is doing, sent to the FieldViewer every loop. Same idea as
 * Gluten Free's ComputerDebugging messages: plain text records, one per line.
 *
 * <pre>
 * F,seq,time          frame header
 * RT,name             route being run
 * ST,text             status text
 * R,x,y,heading       robot pose (in, rad)
 * PW,fwd,left,turn    drive powers
 * FP,x,y,radius       follow point + lookahead circle
 * PT,x,y,radius       point the robot is facing + its circle
 * CP,x,y              closest point on the path
 * PA,x1,y1,x2,y2,...  path being followed
 * </pre>
 */
public final class ViewerFrame {
    public long seq;
    public double time;
    public String routeName = "";
    public String status = "";
    public Pose2d pose;
    public PoseVelocity2d powers;
    public Vector2d followPoint, pointToPoint, clippedPoint;
    public double followRadius, pointRadius;
    public List<Vector2d> path = new ArrayList<>();

    /** Fills in the pure pursuit parts from a follower update. */
    public ViewerFrame withFollower(PurePursuitFollower follower,
                                    PurePursuitFollower.Output output) {
        if (output != null) {
            powers = output.powers;
            followPoint = output.followPoint;
            pointToPoint = output.pointToPoint;
            clippedPoint = output.clippedPoint;
            followRadius = output.followRadius;
            pointRadius = output.pointRadius;
        }
        if (follower != null) {
            path = new ArrayList<>(follower.getPathPoints());
        }
        return this;
    }

    public String encode() {
        StringBuilder sb = new StringBuilder(512);
        sb.append("F,").append(seq).append(',').append(num(time)).append('\n');
        sb.append("RT,").append(clean(routeName)).append('\n');
        sb.append("ST,").append(clean(status)).append('\n');
        if (pose != null) {
            sb.append("R,").append(num(pose.position.x)).append(',').append(num(pose.position.y))
                    .append(',').append(num(pose.heading.toDouble())).append('\n');
        }
        if (powers != null) {
            sb.append("PW,").append(num(powers.linearVel.x)).append(',')
                    .append(num(powers.linearVel.y)).append(',').append(num(powers.angVel)).append('\n');
        }
        if (followPoint != null) {
            sb.append("FP,").append(num(followPoint.x)).append(',').append(num(followPoint.y))
                    .append(',').append(num(followRadius)).append('\n');
        }
        if (pointToPoint != null) {
            sb.append("PT,").append(num(pointToPoint.x)).append(',').append(num(pointToPoint.y))
                    .append(',').append(num(pointRadius)).append('\n');
        }
        if (clippedPoint != null) {
            sb.append("CP,").append(num(clippedPoint.x)).append(',').append(num(clippedPoint.y))
                    .append('\n');
        }
        if (!path.isEmpty()) {
            sb.append("PA");
            for (Vector2d p : path) {
                sb.append(',').append(num(p.x)).append(',').append(num(p.y));
            }
            sb.append('\n');
        }
        return sb.toString();
    }

    /** Parses a frame, skipping any lines it doesn't understand. */
    public static ViewerFrame decode(String text) {
        ViewerFrame frame = new ViewerFrame();
        for (String line : text.split("\n")) {
            int comma = line.indexOf(',');
            if (comma < 0) {
                continue;
            }
            String type = line.substring(0, comma);
            String rest = line.substring(comma + 1);
            try {
                switch (type) {
                    case "RT":
                        frame.routeName = rest;
                        break;
                    case "ST":
                        frame.status = rest;
                        break;
                    default:
                        decodeNumbers(frame, type, rest.split(","));
                }
            } catch (RuntimeException e) {
                // a corrupt record shouldn't take down the whole frame
            }
        }
        return frame;
    }

    private static void decodeNumbers(ViewerFrame frame, String type, String[] parts) {
        double[] v = new double[parts.length];
        for (int i = 0; i < parts.length; i++) {
            v[i] = Double.parseDouble(parts[i]);
        }
        switch (type) {
            case "F":
                frame.seq = (long) v[0];
                frame.time = v[1];
                break;
            case "R":
                frame.pose = new Pose2d(v[0], v[1], v[2]);
                break;
            case "PW":
                frame.powers = new PoseVelocity2d(new Vector2d(v[0], v[1]), v[2]);
                break;
            case "FP":
                frame.followPoint = new Vector2d(v[0], v[1]);
                frame.followRadius = v[2];
                break;
            case "PT":
                frame.pointToPoint = new Vector2d(v[0], v[1]);
                frame.pointRadius = v[2];
                break;
            case "CP":
                frame.clippedPoint = new Vector2d(v[0], v[1]);
                break;
            case "PA":
                List<Vector2d> path = new ArrayList<>();
                for (int i = 0; i + 1 < v.length; i += 2) {
                    path.add(new Vector2d(v[i], v[i + 1]));
                }
                frame.path = path;
                break;
            default:
                break;
        }
    }

    private static String num(double value) {
        return String.format(Locale.US, "%.2f", value);
    }

    private static String clean(String text) {
        return text == null ? "" : text.replace('\n', ' ');
    }
}
