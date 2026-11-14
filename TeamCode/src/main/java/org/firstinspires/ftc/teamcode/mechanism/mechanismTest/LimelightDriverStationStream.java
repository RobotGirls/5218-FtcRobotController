package org.firstinspires.ftc.teamcode.mechanism.mechanismTest;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Supplies individual Limelight MJPEG frames to the FTC Driver Station Camera Stream screen.
 * The Driver Station requests one frame per tap, so this opens the MJPEG feed only long enough
 * to decode its first JPEG frame.
 */
public final class LimelightDriverStationStream implements CameraStreamSource {
    private static final int MAX_JPEG_BYTES = 5 * 1024 * 1024;

    private final URL streamUrl;
    private final ExecutorService executor = Executors.newSingleThreadExecutor(r -> {
        Thread thread = new Thread(r, "Limelight-driver-station-stream");
        thread.setDaemon(true);
        return thread;
    });

    public LimelightDriverStationStream(String host, int port) {
        try {
            streamUrl = new URL("http", host, port, "/");
        } catch (IOException e) {
            throw new IllegalArgumentException("Invalid Limelight stream address", e);
        }
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        executor.execute(() -> {
            Bitmap bitmap = readFirstFrame();
            if (bitmap != null) {
                continuation.dispatch(consumer -> consumer.accept(bitmap));
            }
        });
    }

    public void stop() {
        executor.shutdownNow();
    }

    private Bitmap readFirstFrame() {
        HttpURLConnection connection = null;
        try {
            connection = (HttpURLConnection) streamUrl.openConnection();
            connection.setConnectTimeout(2500);
            connection.setReadTimeout(3500);
            connection.setUseCaches(false);
            connection.connect();
            if (connection.getResponseCode() != HttpURLConnection.HTTP_OK) {
                return null;
            }
            byte[] jpeg = readJpeg(connection.getInputStream());
            return jpeg == null ? null : BitmapFactory.decodeByteArray(jpeg, 0, jpeg.length);
        } catch (IOException ignored) {
            return null;
        } finally {
            if (connection != null) {
                connection.disconnect();
            }
        }
    }

    private static byte[] readJpeg(InputStream input) throws IOException {
        int previous = -1;
        int current;
        while ((current = input.read()) >= 0) {
            if (previous == 0xff && current == 0xd8) {
                break;
            }
            previous = current;
        }
        if (current < 0) {
            return null;
        }

        ByteArrayOutputStream jpeg = new ByteArrayOutputStream(128 * 1024);
        jpeg.write(0xff);
        jpeg.write(0xd8);
        previous = 0xd8;
        while (jpeg.size() < MAX_JPEG_BYTES && (current = input.read()) >= 0) {
            jpeg.write(current);
            if (previous == 0xff && current == 0xd9) {
                return jpeg.toByteArray();
            }
            previous = current;
        }
        return null;
    }
}
