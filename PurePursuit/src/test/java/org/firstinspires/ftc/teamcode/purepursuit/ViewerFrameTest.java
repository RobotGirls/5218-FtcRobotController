package org.firstinspires.ftc.teamcode.purepursuit;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.junit.Test;

import java.util.Arrays;
import java.util.Locale;

public class ViewerFrameTest {
    @Test
    public void roundTrips() {
        ViewerFrame frame = new ViewerFrame();
        frame.seq = 42;
        frame.time = 1.5;
        frame.routeName = "BLUE Garden Park, with a comma";
        frame.status = "Step 1/2";
        frame.pose = new Pose2d(12.34, -5.67, 1.23);
        frame.powers = new PoseVelocity2d(new Vector2d(0.5, -0.25), 0.1);
        frame.followPoint = new Vector2d(20, 30);
        frame.followRadius = 16;
        frame.pointToPoint = new Vector2d(21, 31);
        frame.pointRadius = 18;
        frame.clippedPoint = new Vector2d(10, -5);
        frame.path = Arrays.asList(new Vector2d(0, 0), new Vector2d(48, 0), new Vector2d(48, 24));

        ViewerFrame decoded = ViewerFrame.decode(frame.encode());

        assertEquals(42, decoded.seq);
        assertEquals(1.5, decoded.time, 1e-9);
        assertEquals(frame.routeName, decoded.routeName);
        assertEquals(frame.status, decoded.status);
        assertEquals(12.34, decoded.pose.position.x, 1e-9);
        assertEquals(-5.67, decoded.pose.position.y, 1e-9);
        assertEquals(1.23, decoded.pose.heading.toDouble(), 1e-9);
        assertEquals(-0.25, decoded.powers.linearVel.y, 1e-9);
        assertEquals(16, decoded.followRadius, 1e-9);
        assertEquals(31, decoded.pointToPoint.y, 1e-9);
        assertEquals(-5, decoded.clippedPoint.y, 1e-9);
        assertEquals(frame.path, decoded.path);
    }

    @Test
    public void usesDotsEvenInCommaDecimalLocales() {
        Locale before = Locale.getDefault();
        try {
            Locale.setDefault(Locale.GERMANY);
            ViewerFrame frame = new ViewerFrame();
            frame.pose = new Pose2d(1.5, 2.5, 0);
            assertEquals(1.5, ViewerFrame.decode(frame.encode()).pose.position.x, 1e-9);
        } finally {
            Locale.setDefault(before);
        }
    }

    @Test
    public void ignoresGarbage() {
        ViewerFrame decoded = ViewerFrame.decode("R,1,2\nnonsense\nXX,1,2,3\nPW,a,b,c\n");
        assertNull(decoded.pose);
        assertNull(decoded.powers);
    }
}
