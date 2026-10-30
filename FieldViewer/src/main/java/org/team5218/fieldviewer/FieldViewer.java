package org.team5218.fieldviewer;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

import org.firstinspires.ftc.teamcode.purepursuit.Route;
import org.firstinspires.ftc.teamcode.purepursuit.RouteSimulation;
import org.firstinspires.ftc.teamcode.purepursuit.ViewerFrame;
import org.firstinspires.ftc.teamcode.purepursuit.paths.Routes;

import java.awt.Color;
import java.awt.Desktop;
import java.awt.Graphics2D;
import java.awt.GraphicsEnvironment;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.net.BindException;
import java.net.URI;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import javax.imageio.ImageIO;

/**
 * FieldViewer backend: runs routes through the real PurePursuitFollower, talks to the robot, and
 * serves the TypeScript UI in FieldViewer/web (opened by the Electron app, or any browser).
 *
 * <pre>
 * cd FieldViewer/web && npm install && npm start             desktop app (starts this for you)
 * ./gradlew :FieldViewer:run                                  then open http://localhost:5218
 * ./gradlew :FieldViewer:run --args="--port 8080 --no-browser"
 * ./gradlew :FieldViewer:run --args="--snapshot all build/snapshots"   pictures, no UI
 * </pre>
 */
public final class FieldViewer {
    /** Team number, so it's easy to remember. */
    public static final int DEFAULT_PORT = 5218;

    private FieldViewer() {}

    public static void main(String[] args) throws IOException {
        if (args.length > 0 && args[0].equals("--snapshot")) {
            snapshot(args);
            return;
        }

        int port = DEFAULT_PORT;
        boolean openBrowser = true;
        File webRoot = null;
        for (int i = 0; i < args.length; i++) {
            if (args[i].equals("--port") && i + 1 < args.length) {
                port = Integer.parseInt(args[++i]);
            } else if (args[i].equals("--no-browser")) {
                openBrowser = false;
            } else if (args[i].equals("--web-root") && i + 1 < args.length) {
                webRoot = new File(args[++i]);
            }
        }
        if (webRoot == null) {
            webRoot = findWebRoot();
        }

        String url = "http://localhost:" + port;
        try {
            new WebServer(port, webRoot).start();
        } catch (BindException e) {
            System.out.println("Port " + port + " is busy: the FieldViewer is probably already running at " + url);
            open(url, openBrowser);
            return;
        }
        System.out.println("FieldViewer backend running at " + url + "  (Ctrl+C to stop)");
        if (webRoot == null) {
            System.out.println("UI not built yet: cd FieldViewer/web && npm install && npm run build");
        }
        open(url, openBrowser);
    }

    /** The built TypeScript UI, whether we were started from the repo root or FieldViewer/. */
    private static File findWebRoot() {
        for (String candidate : new String[] {"web/dist", "FieldViewer/web/dist"}) {
            File dir = new File(candidate);
            if (new File(dir, "index.html").isFile()) {
                return dir;
            }
        }
        return null;
    }

    private static void open(String url, boolean openBrowser) {
        if (!openBrowser || GraphicsEnvironment.isHeadless()) {
            return;
        }
        try {
            if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)) {
                Desktop.getDesktop().browse(new URI(url));
            }
        } catch (Exception e) {
            System.out.println("Open " + url + " in your browser");
        }
    }

    // ---- headless pictures --------------------------------------------------------------------

    /**
     * --snapshot &lt;route name | all&gt; &lt;output .png or folder&gt; [tank|mecanum] [seconds]
     * Simulates a route and saves a picture (at the given time, or when it finishes).
     */
    private static void snapshot(String[] args) throws IOException {
        if (args.length < 3) {
            System.err.println("usage: --snapshot <route name | all> <output.png | folder> [tank|mecanum] [seconds]");
            System.exit(2);
        }
        String which = args[1];
        File out = new File(args[2]);
        boolean holonomic = args.length > 3 && args[3].equalsIgnoreCase("mecanum");
        double at = args.length > 4 ? Double.parseDouble(args[4]) : -1;

        List<Route> routes = new ArrayList<>();
        if (which.equalsIgnoreCase("all")) {
            routes.addAll(Routes.ALL);
            out.mkdirs();
        } else {
            Route route = Routes.byName(which);
            if (route == null) {
                System.err.println("no route named '" + which + "'. Routes:");
                for (Route r : Routes.ALL) {
                    System.err.println("  " + r.name);
                }
                System.exit(2);
            }
            routes.add(route);
        }

        FieldRenderer renderer = new FieldRenderer();
        for (Route route : routes) {
            boolean useHolonomic = holonomic || route.requiresHolonomic;
            RouteSimulation sim = new RouteSimulation(route, useHolonomic);
            if (at > 0) {
                while (!sim.isFinished() && sim.getTime() < at) {
                    sim.loop();
                }
            } else {
                sim.runToEnd(60);
            }

            FieldRenderer.Scene scene = new FieldRenderer.Scene();
            scene.route = route;
            scene.frame = sim.frame();
            scene.trail = sim.getTrail();

            int size = 900, gap = 10, logWidth = 340;
            BufferedImage image = new BufferedImage(size + gap + logWidth + gap, size, BufferedImage.TYPE_INT_RGB);
            Graphics2D g = image.createGraphics();
            g.setColor(Color.BLACK);
            g.fillRect(0, 0, image.getWidth(), image.getHeight());
            renderer.render(g, size, scene);
            renderer.drawLogPanel(g, size + gap, gap, logWidth, size - 2 * gap, simLog(sim, useHolonomic));
            g.dispose();

            File file = out.isDirectory()
                    ? new File(out, route.name.replaceAll("[^A-Za-z0-9]+", "-").replaceAll("-$", "") + ".png")
                    : out;
            ImageIO.write(image, "png", file);
            System.out.println("wrote " + file.getPath());
        }
    }

    private static List<String> simLog(RouteSimulation sim, boolean holonomic) {
        ViewerFrame frame = sim.frame();
        Pose2d pose = frame.pose;
        PoseVelocity2d powers = frame.powers;
        List<String> lines = new ArrayList<>();
        lines.add("Robot Coordinates:");
        lines.add(String.format(Locale.US, "X: %.2f , Y: %.2f", pose.position.x, pose.position.y));
        lines.add(String.format(Locale.US, "Angle: %.2f°", Math.toDegrees(pose.heading.toDouble())));
        lines.add("");
        lines.add(sim.route.name);
        lines.add(String.format(Locale.US, "%s  t = %.2f s", holonomic ? "Mecanum" : "Tank", sim.getTime()));
        lines.add(sim.getStatus());
        if (powers != null) {
            lines.add(String.format(Locale.US, "Power: %.2f %.2f %.2f",
                    powers.linearVel.x, powers.linearVel.y, powers.angVel));
        }
        if (sim.isFinished()) {
            lines.add("");
            lines.add(String.format(Locale.US, "Max off path: %.1f in", sim.getMaxCrossTrackError()));
            lines.add(String.format(Locale.US, "Center contact: %.2f s", sim.getCollisionSeconds()));
            if (!sim.getTimedOutSteps().isEmpty()) {
                lines.add("TIMED OUT: " + sim.getTimedOutSteps());
            }
        }
        return lines;
    }
}
