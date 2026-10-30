package org.firstinspires.ftc.teamcode.purepursuit;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.junit.Test;

public class PurePursuitFollowerTest {
    private static final PoseVelocity2d STOPPED = new PoseVelocity2d(new Vector2d(0, 0), 0);

    @Test
    public void pathStartsAtTheRobot() {
        PurePursuitFollower follower = new PurePursuitFollower(
                PurePursuitPath.builder().to(48, 0).build(), new PurePursuitFollower.Params(), true);
        follower.update(new Pose2d(-5, 3, 0), STOPPED);

        assertEquals(new Vector2d(-5, 3), follower.getPathPoints().get(0));
        assertEquals(new Vector2d(48, 0), follower.getPathPoints().get(1));
    }

    @Test
    public void followPointIsOneLookaheadAhead() {
        PurePursuitFollower follower = new PurePursuitFollower(
                PurePursuitPath.builder().followDistance(16).to(100, 0).build(),
                new PurePursuitFollower.Params(), true);
        PurePursuitFollower.Output out = follower.update(new Pose2d(0, 0, 0), STOPPED);

        assertEquals(16, out.followPoint.x, 1e-6);
        assertEquals(0, out.followPoint.y, 1e-6);
        assertTrue("should drive forward", out.powers.linearVel.x > 0.5);
        assertEquals(0, out.powers.linearVel.y, 1e-6);
        assertEquals(0, out.powers.angVel, 1e-6);
    }

    @Test
    public void outAndBackDoesNotSkipToTheReturnLeg() {
        // the return leg passes right over the robot; Gluten Free's version could jump onto it
        PurePursuitFollower follower = new PurePursuitFollower(
                PurePursuitPath.builder().followDistance(12).to(60, 0).to(0, 0).build(),
                new PurePursuitFollower.Params(), true);
        follower.update(new Pose2d(0, 0, 0), STOPPED);
        PurePursuitFollower.Output out = follower.update(new Pose2d(20, 0, 0), STOPPED);

        assertEquals(0, out.segment);
        assertEquals(32, out.followPoint.x, 1e-6);
        assertTrue(out.powers.linearVel.x > 0);
    }

    @Test
    public void reversedPathDrivesBackwards() {
        PurePursuitFollower follower = new PurePursuitFollower(
                PurePursuitPath.builder().to(-48, 0).reversed().build(),
                new PurePursuitFollower.Params(), false);
        PurePursuitFollower.Output out = follower.update(new Pose2d(0, 0, 0), STOPPED);

        assertTrue("should back up", out.powers.linearVel.x < -0.5);
        assertEquals("already facing the right way", 0, out.powers.angVel, 1e-6);
    }

    @Test
    public void tankNeverStrafes() {
        PurePursuitFollower follower = new PurePursuitFollower(
                PurePursuitPath.builder().to(0, 48).build(), new PurePursuitFollower.Params(), false);
        PurePursuitFollower.Output out = follower.update(new Pose2d(0, 0, 0), STOPPED);

        assertEquals(0, out.powers.linearVel.y, 0);
        assertTrue("should turn left towards the point", out.powers.angVel > 0);
    }

    @Test
    public void mecanumStrafesWithFixedHeading() {
        PurePursuitFollower follower = new PurePursuitFollower(
                PurePursuitPath.builder().to(0, 48).fixedHeading(0).build(),
                new PurePursuitFollower.Params(), true);
        PurePursuitFollower.Output out = follower.update(new Pose2d(0, 0, 0), STOPPED);

        assertTrue("should strafe left", out.powers.linearVel.y > 0.5);
        assertEquals(0, out.powers.angVel, 1e-6);
    }

    @Test
    public void finishesWhenOnTheLastPoint() {
        PurePursuitFollower follower = new PurePursuitFollower(
                PurePursuitPath.builder().to(24, 0).build(), new PurePursuitFollower.Params(), true);
        follower.update(new Pose2d(0, 0, 0), STOPPED);
        PurePursuitFollower.Output out = follower.update(new Pose2d(23.5, 0.2, 0), STOPPED);

        assertTrue(out.isDone());
        assertEquals(0, out.powers.linearVel.x, 0);
    }

    @Test
    public void turnsToEndHeadingAfterArriving() {
        PurePursuitFollower follower = new PurePursuitFollower(
                PurePursuitPath.builder().to(24, 0).endHeading(Math.toRadians(90)).build(),
                new PurePursuitFollower.Params(), false);
        follower.update(new Pose2d(0, 0, 0), STOPPED);
        PurePursuitFollower.Output out = follower.update(new Pose2d(24, 0, 0), STOPPED);

        assertEquals(PurePursuitFollower.Phase.TURNING, out.phase);
        assertTrue(out.powers.angVel > 0);

        out = follower.update(new Pose2d(24, 0, Math.toRadians(89)), STOPPED);
        assertTrue(out.isDone());
    }
}
