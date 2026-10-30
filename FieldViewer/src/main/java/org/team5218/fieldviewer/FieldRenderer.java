package org.team5218.fieldviewer;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.purepursuit.BioBuzzField;
import org.firstinspires.ftc.teamcode.purepursuit.CurvePoint;
import org.firstinspires.ftc.teamcode.purepursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.purepursuit.Route;
import org.firstinspires.ftc.teamcode.purepursuit.ViewerFrame;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.GradientPaint;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.Shape;
import java.awt.Stroke;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.awt.geom.Rectangle2D;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import javax.imageio.ImageIO;

/**
 * Draws the field and everything pure pursuit is doing on it, in the dark "neon" style of the
 * original FieldViewer debug receiver (black field, fine grid, cyan debug lines and points,
 * red-to-yellow position log, cyan robot). Used by the window and by --snapshot, so both match.
 *
 * The dark field is generated from this season's biobuzz-field.png when the viewer starts, so a
 * new season only needs a new field image.
 *
 * Field coordinates are Road Runner's: inches, origin at the center, +x right, +y up.
 */
public final class FieldRenderer {
    /** Everything that can be drawn in one frame. */
    public static final class Scene {
        public Route route;
        public ViewerFrame frame;
        public List<Pose2d> trail = new ArrayList<>();
        public List<Vector2d> sketch = new ArrayList<>();
        public Vector2d mouse;
        public boolean showLookahead = true;
        public boolean showTrail = true;
        public boolean showKeepOut = true;
        public boolean showRoute = true;
        public double robotLength = BioBuzzField.ROBOT_LENGTH;
        public double robotWidth = BioBuzzField.ROBOT_WIDTH;
    }

    // the original FieldViewer palette
    static final Color CYAN = new Color(0, 255, 255);
    static final Color DEBUG_LINE = new Color(0, 255, 255, 153);
    static final Color TEAL = new Color(0, 168, 168);
    static final Color RED = new Color(190, 10, 10);
    static final Color AMBER = new Color(225, 160, 40);
    static final Color WHITE_RING = new Color(255, 255, 255, 220);
    static final Color GRID = new Color(255, 255, 255, 36);
    static final Color SKETCH = new Color(255, 64, 200);
    static final Color LOG_TEXT = new Color(0, 70, 74);

    static final Font MONO = new Font("Courier New", Font.PLAIN, 12);

    private final BufferedImage darkField;
    private BufferedImage scaledField;

    public FieldRenderer() {
        BufferedImage source = loadField();
        darkField = source == null ? null : darken(resize(source, Math.min(2048, source.getWidth())));
    }

    /** This season's field in the dark style (about 2048px), or null if the image is missing. */
    BufferedImage getDarkField() {
        return darkField;
    }

    private static BufferedImage loadField() {
        try (InputStream in = FieldRenderer.class.getResourceAsStream("/biobuzz-field.png")) {
            if (in != null) {
                return ImageIO.read(in);
            }
        } catch (IOException ignored) {
            // fall through to the plain grid
        }
        return null;
    }

    /**
     * Turns the light field drawing into the dark style: white floor goes black, black lines go
     * gray, and the alliance colors become the muted teal / red of the original dark field.
     */
    static BufferedImage darken(BufferedImage light) {
        int w = light.getWidth(), h = light.getHeight();
        int[] px = light.getRGB(0, 0, w, h, null, 0, w);
        for (int i = 0; i < px.length; i++) {
            int r = (px[i] >> 16) & 0xFF, g = (px[i] >> 8) & 0xFF, b = px[i] & 0xFF;
            int max = Math.max(r, Math.max(g, b)), min = Math.min(r, Math.min(g, b));
            int sat = max - min;
            int out;
            if (sat < 50) {
                int v = (int) ((255 - (r + g + b) / 3.0) * 0.30);
                out = (v << 16) | (v << 8) | v;
            } else {
                double k = Math.max(0.45, Math.min(1, sat / 170.0));
                Color tint;
                if (b >= r && b >= g) {
                    tint = TEAL;
                } else if (r >= g && g < r * 0.6) {
                    tint = RED;
                } else {
                    tint = AMBER;
                }
                out = ((int) (tint.getRed() * k) << 16) | ((int) (tint.getGreen() * k) << 8)
                        | (int) (tint.getBlue() * k);
            }
            px[i] = 0xFF000000 | out;
        }
        BufferedImage dark = new BufferedImage(w, h, BufferedImage.TYPE_INT_RGB);
        dark.setRGB(0, 0, w, h, px, 0, w);
        return dark;
    }

    // ---- coordinate helpers -------------------------------------------------------------------

    static double toPx(double fieldX, int size) {
        return (fieldX + BioBuzzField.HALF_FIELD) / (2 * BioBuzzField.HALF_FIELD) * size;
    }

    static double toPy(double fieldY, int size) {
        return (BioBuzzField.HALF_FIELD - fieldY) / (2 * BioBuzzField.HALF_FIELD) * size;
    }

    static Vector2d toField(double px, double py, int size) {
        double scale = 2 * BioBuzzField.HALF_FIELD / size;
        return new Vector2d(px * scale - BioBuzzField.HALF_FIELD, BioBuzzField.HALF_FIELD - py * scale);
    }

    // ---- field --------------------------------------------------------------------------------

    public void render(Graphics2D g, int size, Scene scene) {
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
        g.setRenderingHint(RenderingHints.KEY_STROKE_CONTROL, RenderingHints.VALUE_STROKE_PURE);
        double ppi = size / (2 * BioBuzzField.HALF_FIELD);

        if (scaledField == null || scaledField.getWidth() != size) {
            scaledField = buildField(size);
        }
        g.drawImage(scaledField, 0, 0, null);

        if (scene.showKeepOut) {
            drawKeepOut(g, size);
        }
        if (scene.showRoute && scene.route != null) {
            drawRoute(g, size, ppi, scene.route);
        }
        drawSketch(g, size, ppi, scene.sketch);

        ViewerFrame frame = scene.frame;
        if (frame != null && frame.path.size() >= 2) {
            // the path being followed, like the LINE messages in the original viewer
            glow(g, polyline(frame.path, size), DEBUG_LINE, (float) Math.max(3, ppi * 0.7));
        }
        if (scene.showTrail) {
            drawLogPoints(g, size, ppi, scene.trail);
        }
        if (frame != null && frame.pose != null) {
            if (scene.showLookahead) {
                drawLookahead(g, size, ppi, frame);
            }
            boolean hit = BioBuzzField.hitsCenterStructure(frame.pose, scene.robotLength, scene.robotWidth);
            drawRobot(g, size, ppi, frame.pose, scene.robotLength, scene.robotWidth, hit);
        }
        if (scene.mouse != null) {
            drawCrosshair(g, size, scene.mouse);
        }
    }

    /** The dark field at the given size, with the fine alliance tinted grid on top. */
    private BufferedImage buildField(int size) {
        BufferedImage out = new BufferedImage(size, size, BufferedImage.TYPE_INT_RGB);
        Graphics2D g = out.createGraphics();
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        if (darkField != null) {
            g.drawImage(scale(darkField, size), 0, 0, null);
        } else {
            g.setColor(Color.BLACK);
            g.fillRect(0, 0, size, size);
        }

        // fine grid every 4 inches, tinted teal towards the blue side (top) and red towards the
        // red side (bottom), like the original dark field
        int lines = 36;
        int bands = 48;
        g.setStroke(new BasicStroke(1f));
        for (int i = 0; i <= lines; i++) {
            double p = i * size / (double) lines;
            g.setColor(gridColor(p / size));
            g.draw(new Line2D.Double(0, p, size, p));
            for (int band = 0; band < bands; band++) {
                double y0 = band * size / (double) bands, y1 = (band + 1) * size / (double) bands;
                g.setColor(gridColor((y0 + y1) / 2 / size));
                g.draw(new Line2D.Double(p, y0, p, y1));
            }
        }
        g.dispose();
        return out;
    }

    /** Grid color for a height on the field, 0 = top (blue side), 1 = bottom (red side). */
    private static Color gridColor(double t) {
        double blue = Math.max(0, 1 - t / 0.3);
        double red = Math.max(0, (t - 0.7) / 0.3);
        int alpha = (int) (GRID.getAlpha() + 70 * Math.max(blue, red));
        if (blue > 0) {
            return new Color(mix(255, 0, blue), mix(255, 200, blue), mix(255, 200, blue), alpha);
        }
        if (red > 0) {
            return new Color(mix(255, 230, red), mix(255, 30, red), mix(255, 30, red), alpha);
        }
        return GRID;
    }

    private static int mix(int a, int b, double t) {
        return (int) Math.round(a + (b - a) * t);
    }

    /** High quality downscale by repeated halving (a single drawImage looks jagged). */
    private static BufferedImage scale(BufferedImage source, int size) {
        BufferedImage current = source;
        int w = source.getWidth();
        while (w / 2 >= size) {
            w /= 2;
            current = resize(current, w);
        }
        return resize(current, size);
    }

    private static BufferedImage resize(BufferedImage source, int size) {
        BufferedImage out = new BufferedImage(size, size, BufferedImage.TYPE_INT_RGB);
        Graphics2D g = out.createGraphics();
        g.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BILINEAR);
        g.setRenderingHint(RenderingHints.KEY_RENDERING, RenderingHints.VALUE_RENDER_QUALITY);
        g.setColor(Color.WHITE);
        g.fillRect(0, 0, size, size);
        g.drawImage(source, 0, 0, size, size, null);
        g.dispose();
        return out;
    }

    // ---- overlays -----------------------------------------------------------------------------

    private void drawKeepOut(Graphics2D g, int size) {
        double x0 = toPx(BioBuzzField.CENTER_MIN_X, size);
        double y0 = toPy(BioBuzzField.CENTER_MAX_Y, size);
        double x1 = toPx(BioBuzzField.CENTER_MAX_X, size);
        double y1 = toPy(BioBuzzField.CENTER_MIN_Y, size);
        g.setColor(new Color(255, 60, 60, 110));
        g.setStroke(new BasicStroke(1.5f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 10,
                new float[] {6, 6}, 0));
        g.draw(new Rectangle2D.Double(x0, y0, x1 - x0, y1 - y0));
    }

    private void drawRoute(Graphics2D g, int size, double ppi, Route route) {
        Color color = allianceColor(route);
        Vector2d at = route.startPose.position;
        Stroke dashed = new BasicStroke((float) Math.max(1.5, ppi * 0.35), BasicStroke.CAP_ROUND,
                BasicStroke.JOIN_ROUND, 10, new float[] {(float) (ppi * 2.5), (float) (ppi * 2)}, 0);
        int waypoint = 0;

        drawFootprint(g, size, route.startPose, BioBuzzField.ROBOT_LENGTH, BioBuzzField.ROBOT_WIDTH,
                withAlpha(color, 150), new BasicStroke(1.5f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER,
                        10, new float[] {4, 4}, 0));
        for (Route.Step step : route.steps) {
            if (step instanceof Route.Follow) {
                PurePursuitPath path = ((Route.Follow) step).path;
                List<Vector2d> points = new ArrayList<>();
                points.add(at);
                for (CurvePoint p : path.points) {
                    points.add(new Vector2d(p.x, p.y));
                }
                g.setColor(withAlpha(color, 140));
                g.setStroke(dashed);
                g.draw(polyline(points, size));
                for (int i = 1; i < points.size(); i++) {
                    waypoint++;
                    drawWaypoint(g, size, ppi, points.get(i), color, String.valueOf(waypoint));
                }
                at = points.get(points.size() - 1);
            } else if (step instanceof Route.Marker) {
                Route.Marker marker = (Route.Marker) step;
                drawTag(g, size, ppi, at, String.format(Locale.US, "%s %.1fs", marker.name, marker.seconds), color);
            } else if (step instanceof Route.Turn) {
                drawTag(g, size, ppi, at, String.format(Locale.US, "turn %.0f°",
                        Math.toDegrees(((Route.Turn) step).heading)), color);
            }
        }
    }

    private void drawSketch(Graphics2D g, int size, double ppi, List<Vector2d> sketch) {
        if (sketch.isEmpty()) {
            return;
        }
        if (sketch.size() >= 2) {
            glow(g, polyline(sketch, size), withAlpha(SKETCH, 200), (float) Math.max(2, ppi * 0.5));
        }
        for (int i = 0; i < sketch.size(); i++) {
            drawWaypoint(g, size, ppi, sketch.get(i), SKETCH, i == 0 ? "S" : String.valueOf(i));
        }
    }

    /** The robot's past positions as dots going from red (oldest) to yellow (newest). */
    private void drawLogPoints(Graphics2D g, int size, double ppi, List<Pose2d> trail) {
        // like the original's log points, skip ones within 1.5 in of the last one drawn
        List<Vector2d> points = new ArrayList<>();
        for (Pose2d pose : trail) {
            if (points.isEmpty() || dist(points.get(points.size() - 1), pose.position) >= 1.5) {
                points.add(pose.position);
            }
        }
        double r = Math.max(2, ppi * 0.55);
        for (int i = 0; i < points.size(); i++) {
            double t = points.size() == 1 ? 1 : (double) i / (points.size() - 1);
            g.setColor(new Color(1f, (float) t, 0f, 0.9f));
            double px = toPx(points.get(i).x, size), py = toPy(points.get(i).y, size);
            g.fill(new Ellipse2D.Double(px - r, py - r, 2 * r, 2 * r));
        }
    }

    private void drawLookahead(Graphics2D g, int size, double ppi, ViewerFrame frame) {
        Vector2d robot = frame.pose.position;
        double rx = toPx(robot.x, size), ry = toPy(robot.y, size);

        if (frame.followPoint != null && frame.followRadius > 0) {
            double r = frame.followRadius * ppi;
            g.setColor(new Color(0, 255, 255, 22));
            g.fill(new Ellipse2D.Double(rx - r, ry - r, 2 * r, 2 * r));
            g.setColor(new Color(0, 255, 255, 110));
            g.setStroke(new BasicStroke(1.5f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 10,
                    new float[] {5, 5}, 0));
            g.draw(new Ellipse2D.Double(rx - r, ry - r, 2 * r, 2 * r));
        }
        if (frame.clippedPoint != null) {
            ring(g, size, frame.clippedPoint, Math.max(3, ppi * 0.7), new Color(170, 170, 170, 200), 1.5f);
        }
        if (frame.pointToPoint != null) {
            g.setColor(new Color(255, 255, 255, 90));
            g.setStroke(new BasicStroke(1f));
            g.draw(new Line2D.Double(rx, ry, toPx(frame.pointToPoint.x, size), toPy(frame.pointToPoint.y, size)));
            ring(g, size, frame.pointToPoint, Math.max(7, ppi * 2.2), WHITE_RING, 1.5f);
        }
        if (frame.followPoint != null) {
            glow(g, new Line2D.Double(rx, ry, toPx(frame.followPoint.x, size), toPy(frame.followPoint.y, size)),
                    DEBUG_LINE, 2f);
            // the P messages in the original viewer: a cyan ring
            ring(g, size, frame.followPoint, Math.max(5, ppi * 1.4), CYAN, 2.5f);
            double px = toPx(frame.followPoint.x, size), py = toPy(frame.followPoint.y, size);
            g.setColor(CYAN);
            g.fill(new Ellipse2D.Double(px - 2, py - 2, 4, 4));
        }
    }

    /** Neon outline robot, in the spirit of robot.png from the original viewer. */
    private void drawRobot(Graphics2D g, int size, double ppi, Pose2d pose, double length,
                           double width, boolean colliding) {
        Color color = colliding ? new Color(255, 60, 60) : CYAN;
        double l = length / 2, w = width / 2;

        Path2D.Double body = shape(size, pose, new double[][] {{l, w}, {-l, w}, {-l, -w}, {l, -w}});
        g.setColor(new Color(color.getRed(), color.getGreen(), color.getBlue(), 28));
        g.fill(body);
        glow(g, body, color, (float) Math.max(2, ppi * 0.35));

        // four wheels
        double wl = length * 0.26, ww = width * 0.14, inset = 0.06;
        for (int sx = -1; sx <= 1; sx += 2) {
            for (int sy = -1; sy <= 1; sy += 2) {
                double cx = sx * (l - length * inset - wl / 2);
                double cy = sy * (w - width * inset - ww / 2);
                g.setColor(withAlpha(color, 200));
                g.fill(shape(size, pose, new double[][] {
                        {cx + wl / 2, cy + ww / 2}, {cx - wl / 2, cy + ww / 2},
                        {cx - wl / 2, cy - ww / 2}, {cx + wl / 2, cy - ww / 2}}));
            }
        }

        // intake funnel opening towards the front, so you can see which way it faces
        Path2D.Double funnel = shape(size, pose, new double[][] {
                {l, w * 0.62}, {l * 0.25, w * 0.3}, {l * 0.25, -w * 0.3}, {l, -w * 0.62}});
        funnelOpen(funnel);
        glow(g, funnel, color, (float) Math.max(1.5, ppi * 0.25));
        Path2D.Double spine = shape(size, pose, new double[][] {{l * 0.25, 0}, {-l * 0.8, 0}});
        funnelOpen(spine);
        g.setColor(color);
        g.setStroke(new BasicStroke((float) Math.max(1.5, ppi * 0.25)));
        g.draw(spine);
    }

    // ---- log panel ----------------------------------------------------------------------------

    /**
     * The "Log..." panel from the original viewer: black frame, cyan fading to white, and the
     * readouts in Courier New.
     */
    public void drawLogPanel(Graphics2D g, int x, int y, int width, int height, List<String> lines) {
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
        int frame = 4;
        g.setColor(Color.BLACK);
        g.fillRect(x, y, width, height);
        int ix = x + frame, iy = y + frame, iw = width - 2 * frame, ih = height - 2 * frame;
        g.setPaint(new GradientPaint(0, iy, CYAN, 0, iy + ih * 0.42f, Color.WHITE));
        g.fillRect(ix, iy, iw, ih);

        Font title = MONO.deriveFont(Font.PLAIN, Math.max(22f, width / 9.5f));
        g.setFont(title);
        g.setColor(Color.WHITE);
        FontMetrics tm = g.getFontMetrics();
        g.drawString("Log...", ix + 16, iy + 10 + tm.getAscent());

        Font body = MONO.deriveFont(Font.BOLD, Math.max(13f, width / 22f));
        g.setFont(body);
        FontMetrics bm = g.getFontMetrics();
        int maxChars = Math.max(10, (iw - 28) / Math.max(1, bm.charWidth('M')));
        int ty = iy + 10 + tm.getHeight() + 14 + bm.getAscent();
        g.setColor(LOG_TEXT);
        for (String line : lines) {
            for (String part : wrap(line, maxChars)) {
                if (ty > iy + ih - 6) {
                    return;
                }
                g.drawString(part, ix + 16, ty);
                ty += bm.getHeight();
            }
        }
    }

    private static List<String> wrap(String line, int maxChars) {
        List<String> parts = new ArrayList<>();
        String rest = line;
        while (rest.length() > maxChars) {
            int cut = rest.lastIndexOf(' ', maxChars);
            if (cut <= 0) {
                cut = maxChars;
            }
            parts.add(rest.substring(0, cut));
            rest = "  " + rest.substring(cut).trim();
        }
        parts.add(rest);
        return parts;
    }

    // ---- small helpers ------------------------------------------------------------------------

    private void drawWaypoint(Graphics2D g, int size, double ppi, Vector2d p, Color color, String label) {
        double r = Math.max(6, ppi * 1.7);
        double px = toPx(p.x, size), py = toPy(p.y, size);
        g.setColor(Color.BLACK);
        g.fill(new Ellipse2D.Double(px - r, py - r, 2 * r, 2 * r));
        ring(g, size, p, r, color, 1.8f);
        g.setFont(MONO.deriveFont(Font.BOLD, (float) Math.max(10, ppi * 2)));
        FontMetrics fm = g.getFontMetrics();
        g.setColor(brighter(color));
        g.drawString(label, (float) (px - fm.stringWidth(label) / 2.0), (float) (py + fm.getAscent() / 2.0 - 1));
    }

    private void drawTag(Graphics2D g, int size, double ppi, Vector2d at, String text, Color color) {
        double px = toPx(at.x, size), py = toPy(at.y, size);
        g.setFont(MONO.deriveFont(Font.BOLD, (float) Math.max(11, ppi * 2)));
        FontMetrics fm = g.getFontMetrics();
        int pad = 4;
        int w = fm.stringWidth(text) + 2 * pad, h = fm.getHeight() + pad;
        double bx = px + ppi * 2.5, by = py - ppi * 2.5 - h;
        g.setColor(withAlpha(color, 200));
        g.setStroke(new BasicStroke(1.5f));
        g.draw(new Line2D.Double(px, py, bx, by + h));
        g.setColor(Color.BLACK);
        g.fill(new Rectangle2D.Double(bx, by, w, h));
        g.setColor(brighter(color));
        g.draw(new Rectangle2D.Double(bx, by, w, h));
        g.drawString(text, (float) (bx + pad), (float) (by + pad / 2.0 + fm.getAscent()));
    }

    private void drawCrosshair(Graphics2D g, int size, Vector2d mouse) {
        double px = toPx(mouse.x, size), py = toPy(mouse.y, size);
        g.setColor(new Color(0, 255, 255, 70));
        g.setStroke(new BasicStroke(1f));
        g.draw(new Line2D.Double(px, 0, px, size));
        g.draw(new Line2D.Double(0, py, size, py));
    }

    private static void drawFootprint(Graphics2D g, int size, Pose2d pose, double length, double width,
                                      Color color, Stroke stroke) {
        double l = length / 2, w = width / 2;
        g.setColor(color);
        g.setStroke(stroke);
        g.draw(shape(size, pose, new double[][] {{l, w}, {-l, w}, {-l, -w}, {l, -w}}));
    }

    /** A closed shape from robot relative corners (inches). */
    private static Path2D.Double shape(int size, Pose2d pose, double[][] local) {
        Path2D.Double path = new Path2D.Double();
        for (int i = 0; i < local.length; i++) {
            Vector2d p = pose.times(new Vector2d(local[i][0], local[i][1]));
            if (i == 0) {
                path.moveTo(toPx(p.x, size), toPy(p.y, size));
            } else {
                path.lineTo(toPx(p.x, size), toPy(p.y, size));
            }
        }
        path.closePath();
        return path;
    }

    /** Drops the closing segment so a shape draws as an open polyline. */
    private static void funnelOpen(Path2D.Double path) {
        Path2D.Double open = new Path2D.Double();
        double[] c = new double[6];
        boolean first = true;
        for (java.awt.geom.PathIterator it = path.getPathIterator(null); !it.isDone(); it.next()) {
            int type = it.currentSegment(c);
            if (type == java.awt.geom.PathIterator.SEG_CLOSE) {
                continue;
            }
            if (first) {
                open.moveTo(c[0], c[1]);
                first = false;
            } else {
                open.lineTo(c[0], c[1]);
            }
        }
        path.reset();
        path.append(open, false);
    }

    /** Draws a shape with a soft wide halo under it, for the neon look. */
    private static void glow(Graphics2D g, Shape shape, Color color, float width) {
        g.setStroke(new BasicStroke(width * 3.2f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
        g.setColor(withAlpha(color, Math.min(color.getAlpha(), 40)));
        g.draw(shape);
        g.setStroke(new BasicStroke(width, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
        g.setColor(color);
        g.draw(shape);
    }

    private static void ring(Graphics2D g, int size, Vector2d p, double r, Color color, float width) {
        double px = toPx(p.x, size), py = toPy(p.y, size);
        g.setColor(color);
        g.setStroke(new BasicStroke(width));
        g.draw(new Ellipse2D.Double(px - r, py - r, 2 * r, 2 * r));
    }

    private static Path2D.Double polyline(List<Vector2d> points, int size) {
        Path2D.Double path = new Path2D.Double();
        for (int i = 0; i < points.size(); i++) {
            double px = toPx(points.get(i).x, size), py = toPy(points.get(i).y, size);
            if (i == 0) {
                path.moveTo(px, py);
            } else {
                path.lineTo(px, py);
            }
        }
        return path;
    }

    private static double dist(Vector2d a, Vector2d b) {
        return Math.hypot(a.x - b.x, a.y - b.y);
    }

    static Color allianceColor(Route route) {
        if (route == null) {
            return new Color(200, 200, 200);
        }
        switch (route.alliance) {
            case RED:
                return new Color(255, 70, 70);
            case BLUE:
                return new Color(40, 220, 220);
            default:
                return new Color(200, 200, 200);
        }
    }

    private static Color brighter(Color c) {
        return new Color(Math.min(255, c.getRed() + 40), Math.min(255, c.getGreen() + 40),
                Math.min(255, c.getBlue() + 40));
    }

    private static Color withAlpha(Color c, int alpha) {
        return new Color(c.getRed(), c.getGreen(), c.getBlue(), alpha);
    }
}
