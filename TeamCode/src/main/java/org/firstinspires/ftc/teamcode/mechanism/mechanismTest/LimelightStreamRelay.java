package org.firstinspires.ftc.teamcode.mechanism.mechanismTest;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * TCP relay from the robot Wi-Fi network to the Limelight's USB-Ethernet MJPEG server.
 * FieldViewer connects to the Control Hub's relay port; the OpMode process connects onward to
 * the camera at 172.29.x.1:5800. Video bytes never share the pure-pursuit UDP telemetry channel.
 */
public final class LimelightStreamRelay {
    private final String limelightHost;
    private final int limelightPort;
    private final int relayPort;
    private final Set<Socket> sockets = Collections.synchronizedSet(new HashSet<Socket>());

    private volatile boolean running;
    private volatile String status = "stopped";
    private ServerSocket serverSocket;
    private ExecutorService executor;

    public LimelightStreamRelay(String limelightHost, int limelightPort, int relayPort) {
        this.limelightHost = limelightHost;
        this.limelightPort = limelightPort;
        this.relayPort = relayPort;
    }

    public synchronized void start() {
        if (running) {
            return;
        }
        try {
            serverSocket = new ServerSocket();
            serverSocket.setReuseAddress(true);
            serverSocket.bind(new InetSocketAddress(relayPort));
            executor = Executors.newCachedThreadPool(r -> {
                Thread thread = new Thread(r, "Limelight-stream-relay");
                thread.setDaemon(true);
                return thread;
            });
            running = true;
            status = "ready on port " + relayPort;
            executor.execute(this::acceptConnections);
        } catch (IOException e) {
            status = "could not open port " + relayPort + ": " + e.getMessage();
            closeServer();
        }
    }

    public synchronized void stop() {
        running = false;
        closeServer();
        synchronized (sockets) {
            for (Socket socket : sockets) {
                close(socket);
            }
            sockets.clear();
        }
        if (executor != null) {
            executor.shutdownNow();
            executor = null;
        }
        status = "stopped";
    }

    public String getStatus() {
        return status;
    }

    private void acceptConnections() {
        ServerSocket listeningSocket = serverSocket;
        while (running) {
            try {
                Socket viewer = listeningSocket.accept();
                sockets.add(viewer);
                executor.execute(() -> relay(viewer));
            } catch (IOException e) {
                if (running) {
                    status = "accept failed: " + e.getMessage();
                }
            }
        }
    }

    private void relay(Socket viewer) {
        Socket camera = new Socket();
        sockets.add(camera);
        try {
            camera.connect(new InetSocketAddress(limelightHost, limelightPort), 2500);
            camera.setTcpNoDelay(true);
            viewer.setTcpNoDelay(true);
            status = "streaming from " + limelightHost + ":" + limelightPort;

            executor.execute(() -> copy(viewer, camera));
            copy(camera, viewer);
        } catch (IOException e) {
            status = "camera unavailable at " + limelightHost + ":" + limelightPort;
        } finally {
            close(viewer);
            close(camera);
            sockets.remove(viewer);
            sockets.remove(camera);
            if (running) {
                status = "ready on port " + relayPort;
            }
        }
    }

    private static void copy(Socket from, Socket to) {
        byte[] buffer = new byte[16 * 1024];
        try {
            InputStream input = from.getInputStream();
            OutputStream output = to.getOutputStream();
            int count;
            while ((count = input.read(buffer)) >= 0) {
                output.write(buffer, 0, count);
                output.flush();
            }
        } catch (IOException ignored) {
            // The viewer, OpMode, or camera closed the stream.
        }
    }

    private void closeServer() {
        if (serverSocket != null) {
            try {
                serverSocket.close();
            } catch (IOException ignored) {
                // already closed
            }
            serverSocket = null;
        }
    }

    private static void close(Socket socket) {
        try {
            socket.close();
        } catch (IOException ignored) {
            // already closed
        }
    }
}
