package org.team5218.fieldviewer;

import org.firstinspires.ftc.teamcode.purepursuit.FieldViewerLink;
import org.firstinspires.ftc.teamcode.purepursuit.ViewerFrame;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.net.SocketTimeoutException;
import java.nio.charset.StandardCharsets;
import java.util.function.Consumer;

/**
 * Laptop side of the robot link. Says hello to the robot about twice a second (that's how the
 * robot learns where to send frames) and hands every frame it gets back to a listener.
 *
 * Connect the laptop to the robot's wifi first. Control Hub: 192.168.43.1,
 * phone Robot Controller: 192.168.49.1.
 */
final class LiveConnection {
    private static final long HELLO_PERIOD_MS = 500;

    private volatile boolean running;
    private volatile String status = "Not connected";
    private volatile long lastFrameMs;
    private volatile double framesPerSecond;
    private Thread thread;
    private DatagramSocket socket;

    synchronized void start(final String host, final Consumer<ViewerFrame> listener) {
        stop();
        running = true;
        status = "Saying hello to " + host + "...";
        thread = new Thread(() -> run(host, listener), "FieldViewer-live");
        thread.setDaemon(true);
        thread.start();
    }

    synchronized void stop() {
        running = false;
        if (socket != null) {
            socket.close();
        }
        if (thread != null) {
            thread.interrupt();
            thread = null;
        }
        status = "Not connected";
    }

    boolean isRunning() {
        return running;
    }

    String getStatus() {
        if (running && lastFrameMs > 0) {
            long age = System.currentTimeMillis() - lastFrameMs;
            if (age < 1000) {
                return String.format("Receiving %.0f frames/s", framesPerSecond);
            }
            return String.format("No frames for %.0f s (is an OpMode running?)", age / 1000.0);
        }
        return status;
    }

    private void run(String host, Consumer<ViewerFrame> listener) {
        try (DatagramSocket s = new DatagramSocket()) {
            synchronized (this) {
                socket = s;
            }
            s.setSoTimeout(100);
            InetSocketAddress robot = new InetSocketAddress(host, FieldViewerLink.ROBOT_PORT);
            if (robot.isUnresolved()) {
                status = "Can't find " + host;
                running = false;
                return;
            }
            byte[] hello = FieldViewerLink.HELLO.getBytes(StandardCharsets.UTF_8);
            byte[] buffer = new byte[64 * 1024];
            long lastHello = 0;
            long windowStart = System.currentTimeMillis();
            int framesInWindow = 0;

            while (running) {
                long now = System.currentTimeMillis();
                if (now - lastHello > HELLO_PERIOD_MS) {
                    try {
                        s.send(new DatagramPacket(hello, hello.length, robot));
                    } catch (IOException e) {
                        status = "Can't reach " + host + " (" + e.getMessage() + ")";
                    }
                    lastHello = now;
                }
                try {
                    DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
                    s.receive(packet);
                    String text = new String(packet.getData(), 0, packet.getLength(), StandardCharsets.UTF_8);
                    listener.accept(ViewerFrame.decode(text));
                    lastFrameMs = System.currentTimeMillis();
                    framesInWindow++;
                } catch (SocketTimeoutException ignored) {
                    // nothing this time, loop around and maybe say hello again
                }
                if (now - windowStart >= 1000) {
                    framesPerSecond = framesInWindow * 1000.0 / (now - windowStart);
                    framesInWindow = 0;
                    windowStart = now;
                }
            }
        } catch (IOException e) {
            if (running) {
                status = "Connection error: " + e.getMessage();
            }
        } finally {
            running = false;
        }
    }
}
