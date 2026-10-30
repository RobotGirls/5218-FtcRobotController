package org.firstinspires.ftc.teamcode.purepursuit;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketAddress;
import java.net.SocketTimeoutException;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Robot side of the FieldViewer connection (like Gluten Free's UdpServer).
 *
 * Instead of hard coding the laptop's IP address, the robot listens on {@link #ROBOT_PORT} and
 * the FieldViewer sends {@link #HELLO} about twice a second. The robot streams frames back to
 * whoever said hello most recently, and stops on its own a few seconds after the laptop goes
 * away. Nothing is sent when no viewer is connected.
 *
 * The socket lives on one background thread for the whole app, so it survives from one OpMode
 * to the next without "address already in use" errors.
 */
public final class FieldViewerLink {
    public static final int ROBOT_PORT = 11116;
    public static final String HELLO = "FIELDVIEWER_HELLO";
    private static final long VIEWER_TIMEOUT_MS = 3000;

    private static FieldViewerLink instance;

    private final AtomicReference<String> pending = new AtomicReference<>();
    private volatile SocketAddress viewer;
    private volatile long lastHelloMs;
    private volatile String error;
    private long seq;

    private FieldViewerLink() {}

    /** The shared link, started the first time it's needed. */
    public static synchronized FieldViewerLink get() {
        if (instance == null) {
            instance = new FieldViewerLink();
            Thread thread = new Thread(instance::run, "FieldViewerLink");
            thread.setDaemon(true);
            thread.start();
        }
        return instance;
    }

    public boolean isViewerConnected() {
        return viewer != null && System.currentTimeMillis() - lastHelloMs < VIEWER_TIMEOUT_MS;
    }

    /** Why the link isn't working, or null if it's fine. */
    public String getError() {
        return error;
    }

    /** Queues a frame to send. Never blocks; if frames come in faster than they go out, only the newest is sent. */
    public void publish(ViewerFrame frame) {
        if (!isViewerConnected()) {
            return;
        }
        synchronized (this) {
            frame.seq = seq++;
        }
        pending.set(frame.encode());
    }

    private void run() {
        try (DatagramSocket socket = new DatagramSocket(ROBOT_PORT)) {
            socket.setSoTimeout(10);
            byte[] buffer = new byte[256];
            while (!Thread.currentThread().isInterrupted()) {
                try {
                    DatagramPacket hello = new DatagramPacket(buffer, buffer.length);
                    socket.receive(hello);
                    String text = new String(hello.getData(), 0, hello.getLength(), StandardCharsets.UTF_8);
                    if (text.startsWith(HELLO)) {
                        viewer = hello.getSocketAddress();
                        lastHelloMs = System.currentTimeMillis();
                    }
                } catch (SocketTimeoutException ignored) {
                    // no hello this time around, carry on
                }

                String message = pending.getAndSet(null);
                SocketAddress target = viewer;
                if (message != null && target != null && isViewerConnected()) {
                    byte[] bytes = message.getBytes(StandardCharsets.UTF_8);
                    try {
                        socket.send(new DatagramPacket(bytes, bytes.length, target));
                    } catch (IOException e) {
                        // e.g. the laptop dropped off wifi; keep listening for the next hello
                        error = e.toString();
                    }
                }
            }
        } catch (IOException e) {
            error = e.toString();
        }
    }
}
