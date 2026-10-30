package org.firstinspires.ftc.teamcode.purepursuit;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Vector2d;

import org.junit.Test;

import java.util.List;

public class PurePursuitMathTest {
    private static final double EPS = 1e-9;

    @Test
    public void angleWrapStaysInRange() {
        assertEquals(0, PurePursuitMath.angleWrap(2 * Math.PI), EPS);
        assertEquals(-Math.PI / 2, PurePursuitMath.angleWrap(3 * Math.PI / 2), EPS);
        assertEquals(Math.PI / 2, PurePursuitMath.angleWrap(-3 * Math.PI / 2), EPS);
        assertEquals(0.1, PurePursuitMath.angleWrap(0.1 - 4 * Math.PI), EPS);
    }

    @Test
    public void verticalSegmentCrossesCircleTwice() {
        // the slope based version needed a hack for this case
        List<Vector2d> hits = PurePursuitMath.lineCircleIntersection(
                new Vector2d(0, 0), 5, new Vector2d(3, -10), new Vector2d(3, 10));
        assertEquals(2, hits.size());
        assertEquals(3, hits.get(0).x, EPS);
        assertEquals(-4, hits.get(0).y, EPS);
        assertEquals(3, hits.get(1).x, EPS);
        assertEquals(4, hits.get(1).y, EPS);
    }

    @Test
    public void horizontalSegmentCrossesCircleTwice() {
        List<Vector2d> hits = PurePursuitMath.lineCircleIntersection(
                new Vector2d(10, 10), 5, new Vector2d(0, 13), new Vector2d(20, 13));
        assertEquals(2, hits.size());
        assertEquals(6, hits.get(0).x, EPS);
        assertEquals(14, hits.get(1).x, EPS);
    }

    @Test
    public void onlyPointsOnTheSegmentCount() {
        // starts inside the circle: only the exit point is on the segment
        List<Vector2d> hits = PurePursuitMath.lineCircleIntersection(
                new Vector2d(0, 0), 5, new Vector2d(0, 0), new Vector2d(10, 0));
        assertEquals(1, hits.size());
        assertEquals(5, hits.get(0).x, EPS);

        // entirely inside the circle
        assertTrue(PurePursuitMath.lineCircleIntersection(
                new Vector2d(0, 0), 5, new Vector2d(-1, 0), new Vector2d(1, 0)).isEmpty());

        // misses the circle
        assertTrue(PurePursuitMath.lineCircleIntersection(
                new Vector2d(0, 0), 5, new Vector2d(-10, 6), new Vector2d(10, 6)).isEmpty());
    }

    @Test
    public void tangentSegmentTouchesOnce() {
        List<Vector2d> hits = PurePursuitMath.lineCircleIntersection(
                new Vector2d(0, 0), 5, new Vector2d(-10, 5), new Vector2d(10, 5));
        assertEquals(1, hits.size());
        assertEquals(0, hits.get(0).x, 1e-6);
    }

    @Test
    public void clipToSegmentClampsToEnds() {
        Vector2d a = new Vector2d(0, 0);
        Vector2d b = new Vector2d(10, 0);
        assertEquals(4, PurePursuitMath.clipToSegment(new Vector2d(4, 7), a, b).x, EPS);
        assertEquals(0, PurePursuitMath.clipToSegment(new Vector2d(-5, 3), a, b).x, EPS);
        assertEquals(10, PurePursuitMath.clipToSegment(new Vector2d(15, -3), a, b).x, EPS);
    }
}
