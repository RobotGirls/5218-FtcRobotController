package org.team5218.fieldviewer;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;

import org.firstinspires.ftc.teamcode.purepursuit.BioBuzzField;
import org.firstinspires.ftc.teamcode.purepursuit.CurvePoint;
import org.firstinspires.ftc.teamcode.purepursuit.DriveSimulator;
import org.firstinspires.ftc.teamcode.purepursuit.PurePursuitFollower;
import org.firstinspires.ftc.teamcode.purepursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.purepursuit.Route;
import org.firstinspires.ftc.teamcode.purepursuit.RouteSimulation;
import org.firstinspires.ftc.teamcode.purepursuit.ViewerFrame;
import org.firstinspires.ftc.teamcode.purepursuit.paths.Routes;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

import javax.imageio.ImageIO;

/**
 * Serves the FieldViewer web UI (src/main/resources/web) and its API on localhost:
 *
 * <pre>
 * GET  /api/routes            every route in Routes.ALL plus field geometry
 * POST /api/simulate          {"route": name | "sketch": {...}, "holonomic": bool} -> full timeline
 * GET  /api/live/events       Server-Sent Events: live robot frames + connection status
 * POST /api/live/connect      {"host": "192.168.43.1"}
 * POST /api/live/disconnect
 * GET  /field-dark.png        this season's field, darkened
 * GET  /api/defaults          default follower / simulated drive tuning, for the tuning panel
 * GET  /api/health            "ok" once the server is up (the Electron app waits for this)
 * GET  /*                     the built TypeScript UI from FieldViewer/web/dist
 * </pre>
 *
 * Only binds to localhost, so nobody else on the robot's wifi can reach it.
 */
final class WebServer {
    private static final double MAX_SIM_SECONDS = 60;

    private final HttpServer server;
    private final File webRoot;
    private final byte[] darkFieldPng;
    private final LiveConnection live = new LiveConnection();
    private final AtomicReference<ViewerFrame> liveFrame = new AtomicReference<>();
    private final AtomicLong liveVersion = new AtomicLong();
    private volatile String liveHost = "";

    /** @param webRoot folder holding the built UI (index.html), or null if it isn't built */
    WebServer(int port, File webRoot) throws IOException {
        this.webRoot = webRoot == null ? null : webRoot.getCanonicalFile();
        darkFieldPng = png(new FieldRenderer().getDarkField());

        server = HttpServer.create(new InetSocketAddress(InetAddress.getLoopbackAddress(), port), 0);
        server.setExecutor(Executors.newCachedThreadPool(r -> {
            Thread t = new Thread(r, "FieldViewer-http");
            t.setDaemon(true);
            return t;
        }));
        server.createContext("/api/health", ex -> safely(ex, () ->
                send(ex, 200, "text/plain", "ok".getBytes(StandardCharsets.UTF_8))));
        server.createContext("/api/defaults", ex -> safely(ex, () -> sendJson(ex, defaultsJson())));
        server.createContext("/api/routes", ex -> safely(ex, () -> sendJson(ex, routesJson())));
        server.createContext("/api/simulate", ex -> safely(ex, () -> sendJson(ex, simulate(readJson(ex)))));
        server.createContext("/api/live/connect", ex -> safely(ex, () -> {
            Object host = readJson(ex).get("host");
            liveHost = host == null ? "192.168.43.1" : host.toString().trim();
            live.start(liveHost, frame -> {
                liveFrame.set(frame);
                liveVersion.incrementAndGet();
            });
            sendJson(ex, "{\"ok\":true}");
        }));
        server.createContext("/api/live/disconnect", ex -> safely(ex, () -> {
            live.stop();
            sendJson(ex, "{\"ok\":true}");
        }));
        server.createContext("/api/live/events", this::liveEvents);
        server.createContext("/field-dark.png", ex -> safely(ex, () -> send(ex, 200, "image/png", darkFieldPng)));
        server.createContext("/", ex -> safely(ex, () -> staticFile(ex)));
    }

    void start() {
        server.start();
    }

    // ---- routes -------------------------------------------------------------------------------

    private String routesJson() {
        StringBuilder sb = new StringBuilder("{\"routes\":[");
        for (int i = 0; i < Routes.ALL.size(); i++) {
            if (i > 0) {
                sb.append(',');
            }
            sb.append(routeJson(Routes.ALL.get(i)));
        }
        sb.append("],\"field\":{\"half\":").append(Json.num(BioBuzzField.HALF_FIELD))
                .append(",\"center\":[").append(Json.num(BioBuzzField.CENTER_MIN_X)).append(',')
                .append(Json.num(BioBuzzField.CENTER_MIN_Y)).append(',')
                .append(Json.num(BioBuzzField.CENTER_MAX_X)).append(',')
                .append(Json.num(BioBuzzField.CENTER_MAX_Y)).append("],\"robot\":[")
                .append(Json.num(BioBuzzField.ROBOT_LENGTH)).append(',')
                .append(Json.num(BioBuzzField.ROBOT_WIDTH)).append("]}}");
        return sb.toString();
    }

    private static String routeJson(Route route) {
        StringBuilder sb = new StringBuilder("{");
        sb.append("\"name\":").append(Json.quote(route.name))
                .append(",\"description\":").append(Json.quote(route.description))
                .append(",\"alliance\":").append(Json.quote(route.alliance.name()))
                .append(",\"group\":").append(Json.quote(route.name.startsWith("TEST") ? "Test paths"
                        : route.name.equals("Sketch") ? "Sketch" : "BioBuzz autos"))
                .append(",\"requiresHolonomic\":").append(route.requiresHolonomic)
                .append(",\"start\":").append(pose(route.startPose))
                .append(",\"steps\":[");
        for (int i = 0; i < route.steps.size(); i++) {
            if (i > 0) {
                sb.append(',');
            }
            Route.Step step = route.steps.get(i);
            if (step instanceof Route.Follow) {
                PurePursuitPath path = ((Route.Follow) step).path;
                sb.append("{\"type\":\"follow\",\"name\":").append(Json.quote(path.name))
                        .append(",\"reversed\":").append(Math.abs(Math.abs(path.followAngle) - Math.PI) < 1e-6)
                        .append(",\"endHeading\":").append(path.endHeading == null ? "null" : Json.num(path.endHeading))
                        .append(",\"fixedHeading\":").append(path.fixedHeading == null ? "null" : Json.num(path.fixedHeading))
                        .append(",\"speed\":").append(Json.num(path.points.get(0).moveSpeed))
                        .append(",\"lookahead\":").append(Json.num(path.points.get(0).followDistance))
                        .append(",\"points\":[");
                for (int p = 0; p < path.points.size(); p++) {
                    CurvePoint point = path.points.get(p);
                    sb.append(p > 0 ? "," : "").append('[').append(Json.num(point.x)).append(',')
                            .append(Json.num(point.y)).append(']');
                }
                sb.append("]}");
            } else if (step instanceof Route.Marker) {
                Route.Marker marker = (Route.Marker) step;
                sb.append("{\"type\":\"marker\",\"name\":").append(Json.quote(marker.name))
                        .append(",\"seconds\":").append(Json.num(marker.seconds)).append('}');
            } else {
                sb.append("{\"type\":\"turn\",\"heading\":")
                        .append(Json.num(((Route.Turn) step).heading)).append('}');
            }
        }
        return sb.append("]}").toString();
    }

    // ---- simulation ---------------------------------------------------------------------------

    @SuppressWarnings("unchecked")
    private String simulate(Map<String, Object> request) {
        boolean holonomic = Boolean.TRUE.equals(request.get("holonomic"));
        Route route;
        String java = null;
        if (request.get("sketch") instanceof Map) {
            Map<String, Object> s = (Map<String, Object>) request.get("sketch");
            Sketch sketch = new Sketch();
            // each point is [x, y] or [x, y, speed or null, lookahead or null]
            for (Object p : (List<Object>) s.get("points")) {
                List<Object> point = (List<Object>) p;
                sketch.add(((Number) point.get(0)).doubleValue(), ((Number) point.get(1)).doubleValue(),
                        point.size() > 2 ? optionalNumber(point.get(2)) : null,
                        point.size() > 3 ? optionalNumber(point.get(3)) : null);
            }
            sketch.reversed = Boolean.TRUE.equals(s.get("reversed"));
            sketch.name = s.get("name") instanceof String ? (String) s.get("name") : "";
            sketch.startHeading = optionalNumber(s.get("startHeading"));
            sketch.endHeading = optionalNumber(s.get("endHeading"));
            sketch.fixedHeading = optionalNumber(s.get("fixedHeading"));
            if (s.get("moveSpeed") instanceof Number) {
                sketch.moveSpeed = ((Number) s.get("moveSpeed")).doubleValue();
            }
            if (s.get("followDistance") instanceof Number) {
                sketch.followDistance = ((Number) s.get("followDistance")).doubleValue();
            }
            if (!sketch.isUsable()) {
                return "{\"error\":\"a sketch needs a start and at least one waypoint\"}";
            }
            route = sketch.toRoute();
            java = sketch.toJava();
        } else {
            route = Routes.byName(String.valueOf(request.get("route")));
            if (route == null) {
                return "{\"error\":\"no route named " + request.get("route") + "\"}";
            }
        }

        // tuning from the UI's tuning panel; anything not sent keeps its default
        PurePursuitFollower.Params followerParams = new PurePursuitFollower.Params();
        DriveSimulator.Params driveParams = new DriveSimulator.Params();
        applyOverrides(followerParams, request.get("follower"));
        applyOverrides(driveParams, request.get("drive"));
        double[] robot = {BioBuzzField.ROBOT_LENGTH, BioBuzzField.ROBOT_WIDTH};
        if (request.get("robot") instanceof List && ((List<Object>) request.get("robot")).size() == 2) {
            robot[0] = ((Number) ((List<Object>) request.get("robot")).get(0)).doubleValue();
            robot[1] = ((Number) ((List<Object>) request.get("robot")).get(1)).doubleValue();
        }

        RouteSimulation sim = new RouteSimulation(route, holonomic || route.requiresHolonomic,
                followerParams, driveParams);
        List<List<Vector2d>> paths = new ArrayList<>();
        Map<String, Integer> statusIds = new HashMap<>();
        List<String> statuses = new ArrayList<>();
        StringBuilder frames = new StringBuilder();

        int contactFrames = appendFrame(frames, sim, paths, statusIds, statuses, robot);
        while (!sim.isFinished() && sim.getTime() < MAX_SIM_SECONDS) {
            sim.loop();
            frames.append(',');
            contactFrames += appendFrame(frames, sim, paths, statusIds, statuses, robot);
        }

        StringBuilder sb = new StringBuilder("{");
        sb.append("\"route\":").append(routeJson(route))
                .append(",\"holonomic\":").append(sim.holonomic)
                .append(",\"dt\":").append(Json.num(RouteSimulation.LOOP_SECONDS))
                .append(",\"summary\":{\"duration\":").append(Json.num(sim.getRouteDoneTime()))
                .append(",\"settled\":").append(Json.num(sim.getTime()))
                .append(",\"finished\":").append(sim.isFinished())
                .append(",\"maxOffPath\":").append(Json.num(sim.getMaxCrossTrackError()))
                .append(",\"centerContact\":").append(Json.num(contactFrames * RouteSimulation.LOOP_SECONDS))
                .append(",\"timedOut\":[");
        List<String> timedOut = sim.getTimedOutSteps();
        for (int i = 0; i < timedOut.size(); i++) {
            sb.append(i > 0 ? "," : "").append(Json.quote(timedOut.get(i)));
        }
        sb.append("],\"end\":").append(pose(sim.getPose())).append('}');
        sb.append(",\"paths\":[");
        for (int i = 0; i < paths.size(); i++) {
            sb.append(i > 0 ? "," : "").append(points(paths.get(i)));
        }
        sb.append("],\"statuses\":[");
        for (int i = 0; i < statuses.size(); i++) {
            sb.append(i > 0 ? "," : "").append(Json.quote(statuses.get(i)));
        }
        sb.append("],\"frames\":[").append(frames).append(']');
        if (java != null) {
            sb.append(",\"java\":").append(Json.quote(java));
        }
        return sb.append('}').toString();
    }

    /**
     * One timeline entry: [t, x, y, h, fx, fy, fr, px, py, pr, cx, cy, fwd, left, turn,
     * pathIndex, stepIndex, statusIndex, colliding, speed, turnRate]. Follower fields are null
     * during markers. Returns 1 if the robot overlaps the center structure.
     */
    private static int appendFrame(StringBuilder sb, RouteSimulation sim, List<List<Vector2d>> paths,
                                   Map<String, Integer> statusIds, List<String> statuses, double[] robot) {
        ViewerFrame f = sim.frame();
        int pathIndex = -1;
        if (f.path.size() >= 2) {
            pathIndex = paths.isEmpty() || !paths.get(paths.size() - 1).equals(f.path) ? paths.size() : paths.size() - 1;
            if (pathIndex == paths.size()) {
                paths.add(f.path);
            }
        }
        Integer statusIndex = statusIds.get(f.status);
        if (statusIndex == null) {
            statusIndex = statuses.size();
            statuses.add(f.status);
            statusIds.put(f.status, statusIndex);
        }
        boolean colliding = BioBuzzField.hitsCenterStructure(f.pose, robot[0], robot[1]);
        PoseVelocity2d w = f.powers;
        PoseVelocity2d velocity = sim.getVelocity();

        sb.append('[').append(Json.num(f.time)).append(',')
                .append(Json.num(f.pose.position.x)).append(',').append(Json.num(f.pose.position.y)).append(',')
                .append(Json.num(f.pose.heading.toDouble())).append(',');
        appendPoint(sb, f.followPoint, f.followRadius);
        appendPoint(sb, f.pointToPoint, f.pointRadius);
        if (f.clippedPoint == null) {
            sb.append("null,null,");
        } else {
            sb.append(Json.num(f.clippedPoint.x)).append(',').append(Json.num(f.clippedPoint.y)).append(',');
        }
        sb.append(Json.num(w == null ? 0 : w.linearVel.x)).append(',')
                .append(Json.num(w == null ? 0 : w.linearVel.y)).append(',')
                .append(Json.num(w == null ? 0 : w.angVel)).append(',')
                .append(pathIndex).append(',').append(Math.min(sim.getStepIndex(), sim.route.steps.size()))
                .append(',').append(statusIndex).append(',').append(colliding ? 1 : 0)
                .append(',').append(Json.num(velocity.linearVel.norm()))
                .append(',').append(Json.num(velocity.angVel)).append(']');
        return colliding ? 1 : 0;
    }

    private static Double optionalNumber(Object value) {
        return value instanceof Number ? ((Number) value).doubleValue() : null;
    }

    // ---- tuning -------------------------------------------------------------------------------

    private static String defaultsJson() {
        return "{\"follower\":" + paramsJson(new PurePursuitFollower.Params())
                + ",\"drive\":" + paramsJson(new DriveSimulator.Params())
                + ",\"robot\":[" + Json.num(BioBuzzField.ROBOT_LENGTH) + "," + Json.num(BioBuzzField.ROBOT_WIDTH) + "]}";
    }

    /** Every public double field of a Params object, so the UI always matches the Java code. */
    private static String paramsJson(Object params) {
        StringBuilder sb = new StringBuilder("{");
        for (Field field : params.getClass().getFields()) {
            if (field.getType() != double.class || Modifier.isStatic(field.getModifiers())) {
                continue;
            }
            try {
                // full precision: the radian defaults (e.g. 45 degrees) must survive the round trip
                double value = field.getDouble(params);
                sb.append(sb.length() > 1 ? "," : "").append(Json.quote(field.getName())).append(':')
                        .append(Double.isFinite(value) ? String.valueOf(value) : "null");
            } catch (IllegalAccessException ignored) {
                // public fields are always readable
            }
        }
        return sb.append('}').toString();
    }

    /** Sets public double fields from a {"name": number} map, ignoring anything unknown. */
    @SuppressWarnings("unchecked")
    private static void applyOverrides(Object params, Object overrides) {
        if (!(overrides instanceof Map)) {
            return;
        }
        for (Map.Entry<String, Object> entry : ((Map<String, Object>) overrides).entrySet()) {
            if (!(entry.getValue() instanceof Number)) {
                continue;
            }
            try {
                Field field = params.getClass().getField(entry.getKey());
                if (field.getType() == double.class && !Modifier.isStatic(field.getModifiers())) {
                    field.setDouble(params, ((Number) entry.getValue()).doubleValue());
                }
            } catch (NoSuchFieldException | IllegalAccessException ignored) {
                // stale setting from an older version of the panel
            }
        }
    }

    private static void appendPoint(StringBuilder sb, Vector2d p, double radius) {
        if (p == null) {
            sb.append("null,null,null,");
        } else {
            sb.append(Json.num(p.x)).append(',').append(Json.num(p.y)).append(',').append(Json.num(radius)).append(',');
        }
    }

    // ---- live ---------------------------------------------------------------------------------

    /** Streams robot frames (about 30 per second at most) and connection status to the page. */
    private void liveEvents(HttpExchange ex) throws IOException {
        ex.getResponseHeaders().set("Content-Type", "text/event-stream; charset=utf-8");
        ex.getResponseHeaders().set("Cache-Control", "no-cache");
        ex.sendResponseHeaders(200, 0);
        long sentVersion = -1;
        long lastStatusMs = 0;
        try (OutputStream out = ex.getResponseBody()) {
            while (true) {
                long now = System.currentTimeMillis();
                if (now - lastStatusMs > 500) {
                    String status = "{\"running\":" + live.isRunning() + ",\"host\":" + Json.quote(liveHost)
                            + ",\"text\":" + Json.quote(live.getStatus()) + "}";
                    out.write(("event: status\ndata: " + status + "\n\n").getBytes(StandardCharsets.UTF_8));
                    out.flush();
                    lastStatusMs = now;
                }
                long version = liveVersion.get();
                ViewerFrame frame = liveFrame.get();
                if (version != sentVersion && frame != null) {
                    sentVersion = version;
                    out.write(("event: frame\ndata: " + liveFrameJson(frame) + "\n\n").getBytes(StandardCharsets.UTF_8));
                    out.flush();
                }
                Thread.sleep(33);
            }
        } catch (IOException | InterruptedException e) {
            // the page was closed or reloaded
        }
    }

    private static String liveFrameJson(ViewerFrame f) {
        StringBuilder sb = new StringBuilder("{");
        sb.append("\"seq\":").append(f.seq)
                .append(",\"route\":").append(Json.quote(f.routeName))
                .append(",\"status\":").append(Json.quote(f.status))
                .append(",\"pose\":").append(f.pose == null ? "null" : pose(f.pose))
                .append(",\"follow\":").append(f.followPoint == null ? "null"
                        : "[" + Json.num(f.followPoint.x) + "," + Json.num(f.followPoint.y) + "," + Json.num(f.followRadius) + "]")
                .append(",\"pointTo\":").append(f.pointToPoint == null ? "null"
                        : "[" + Json.num(f.pointToPoint.x) + "," + Json.num(f.pointToPoint.y) + "," + Json.num(f.pointRadius) + "]")
                .append(",\"clipped\":").append(f.clippedPoint == null ? "null"
                        : "[" + Json.num(f.clippedPoint.x) + "," + Json.num(f.clippedPoint.y) + "]")
                .append(",\"powers\":").append(f.powers == null ? "null"
                        : "[" + Json.num(f.powers.linearVel.x) + "," + Json.num(f.powers.linearVel.y) + "," + Json.num(f.powers.angVel) + "]")
                .append(",\"path\":").append(points(f.path))
                .append(",\"colliding\":").append(f.pose != null && BioBuzzField.hitsCenterStructure(
                        f.pose, BioBuzzField.ROBOT_LENGTH, BioBuzzField.ROBOT_WIDTH));
        return sb.append('}').toString();
    }

    // ---- helpers ------------------------------------------------------------------------------

    private static String pose(Pose2d p) {
        return "[" + Json.num(p.position.x) + "," + Json.num(p.position.y) + "," + Json.num(p.heading.toDouble()) + "]";
    }

    private static String points(List<Vector2d> points) {
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < points.size(); i++) {
            sb.append(i > 0 ? "," : "").append('[').append(Json.num(points.get(i).x)).append(',')
                    .append(Json.num(points.get(i).y)).append(']');
        }
        return sb.append(']').toString();
    }

    private void staticFile(HttpExchange ex) throws IOException {
        if (webRoot == null) {
            send(ex, 200, "text/html; charset=utf-8", ("<!doctype html><title>FieldViewer</title>"
                    + "<body style='background:#07090c;color:#e6edf3;font:16px system-ui;padding:40px'>"
                    + "<h2>The FieldViewer UI isn't built yet</h2><pre style='color:#22d3ee'>"
                    + "cd FieldViewer/web\nnpm install\nnpm run build</pre>then reload this page.")
                    .getBytes(StandardCharsets.UTF_8));
            return;
        }
        String path = ex.getRequestURI().getPath();
        if (path.equals("/")) {
            path = "/index.html";
        }
        File file = new File(webRoot, path.substring(1)).getCanonicalFile();
        // never serve anything outside the UI folder
        if (!file.toPath().startsWith(webRoot.toPath()) || !file.isFile()) {
            send(ex, 404, "text/plain", "not found".getBytes(StandardCharsets.UTF_8));
            return;
        }
        try (InputStream in = new FileInputStream(file)) {
            send(ex, 200, contentType(path), readAll(in));
        }
    }

    private static String contentType(String path) {
        if (path.endsWith(".html")) {
            return "text/html; charset=utf-8";
        } else if (path.endsWith(".js")) {
            return "text/javascript; charset=utf-8";
        } else if (path.endsWith(".css")) {
            return "text/css; charset=utf-8";
        } else if (path.endsWith(".svg")) {
            return "image/svg+xml";
        } else if (path.endsWith(".png")) {
            return "image/png";
        }
        return "application/octet-stream";
    }

    @SuppressWarnings("unchecked")
    private static Map<String, Object> readJson(HttpExchange ex) throws IOException {
        String body = new String(readAll(ex.getRequestBody()), StandardCharsets.UTF_8).trim();
        if (body.isEmpty()) {
            return new HashMap<>();
        }
        Object parsed = Json.parse(body);
        return parsed instanceof Map ? (Map<String, Object>) parsed : new HashMap<String, Object>();
    }

    private static void sendJson(HttpExchange ex, String json) throws IOException {
        send(ex, 200, "application/json; charset=utf-8", json.getBytes(StandardCharsets.UTF_8));
    }

    private static void send(HttpExchange ex, int code, String type, byte[] body) throws IOException {
        ex.getResponseHeaders().set("Content-Type", type);
        ex.getResponseHeaders().set("Cache-Control", "no-store");
        ex.sendResponseHeaders(code, body.length);
        try (OutputStream out = ex.getResponseBody()) {
            out.write(body);
        }
    }

    private interface Handler {
        void handle() throws IOException;
    }

    /** Turns any exception into a 500 with the message instead of a dropped connection. */
    private static void safely(HttpExchange ex, Handler handler) throws IOException {
        try {
            handler.handle();
        } catch (RuntimeException e) {
            byte[] body = ("{\"error\":" + Json.quote(String.valueOf(e)) + "}").getBytes(StandardCharsets.UTF_8);
            send(ex, 500, "application/json; charset=utf-8", body);
        }
    }

    private static byte[] readAll(InputStream in) throws IOException {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        byte[] buffer = new byte[8192];
        int n;
        while ((n = in.read(buffer)) > 0) {
            out.write(buffer, 0, n);
        }
        return out.toByteArray();
    }

    private static byte[] png(java.awt.image.BufferedImage image) throws IOException {
        if (image == null) {
            return new byte[0];
        }
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        ImageIO.write(image, "png", out);
        return out.toByteArray();
    }
}
