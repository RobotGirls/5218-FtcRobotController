package org.firstinspires.ftc.teamcode.purepursuit;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

import org.firstinspires.ftc.teamcode.purepursuit.paths.TestRoutes;
import org.junit.Test;

/** Runs the auto tuner against a simulated robot whose real numbers differ from the defaults. */
public class AutoTunerTest {
    private static final double LOOP = RouteSimulation.LOOP_SECONDS;

    private static DriveSimulator.Params heavierRobot() {
        DriveSimulator.Params real = new DriveSimulator.Params();
        real.maxForwardVel = 62;
        real.maxStrafeVel = 46;
        real.maxAngVel = Math.toRadians(300);
        real.timeConstant = 0.22;
        real.staticFrictionPower = 0.09;
        return real;
    }

    /** Drives the tuner to the end; returns it so the test can check the results. */
    private static AutoTuner run(DriveSimulator.Params real, boolean holonomic, PurePursuitFollower.Params tuning) {
        Pose2d start = TestRoutes.TUNING_COURSE.startPose;
        DriveSimulator robot = new DriveSimulator(start, holonomic, real);
        AutoTuner tuner = new AutoTuner(start, tuning, holonomic);
        for (double t = 0; t < 400 && !tuner.isDone(); t += LOOP) {
            PoseVelocity2d powers = tuner.update(robot.getPose(), robot.getVelocity(), LOOP);
            for (int i = 0; i < 4; i++) {
                robot.step(powers, LOOP / 4);
            }
            assertFalse("left the field", BioBuzzField.outsideField(robot.getPose(),
                    BioBuzzField.ROBOT_LENGTH, BioBuzzField.ROBOT_WIDTH));
            assertFalse("hit the center structure", BioBuzzField.hitsCenterStructure(robot.getPose(),
                    BioBuzzField.ROBOT_LENGTH, BioBuzzField.ROBOT_WIDTH));
        }
        return tuner;
    }

    @Test
    public void measuresTheRobotAndMatchesTheCodedPathsOnMecanum() {
        DriveSimulator.Params real = heavierRobot();
        PurePursuitFollower.Params tuning = new PurePursuitFollower.Params();
        AutoTuner tuner = run(real, true, tuning);
        for (String line : tuner.getLog()) {
            System.out.println("tuner: " + line);
        }

        assertTrue("didn't finish: " + tuner.getStatus(), tuner.isDone());
        assertFalse(tuner.getStatus(), tuner.isAborted());
        assertEquals(real.maxForwardVel, tuner.measuredDrive.maxForwardVel, real.maxForwardVel * 0.1);
        assertEquals(real.maxStrafeVel, tuner.measuredDrive.maxStrafeVel, real.maxStrafeVel * 0.1);
        assertEquals(real.maxAngVel, tuner.measuredDrive.maxAngVel, real.maxAngVel * 0.1);
        assertEquals(real.timeConstant, tuner.measuredDrive.timeConstant, 0.04);
        assertEquals(real.staticFrictionPower, tuner.measuredDrive.staticFrictionPower, 0.03);
        // a first order lag coasts about speed * timeConstant, so that's the slip to expect
        assertEquals(real.timeConstant, tuning.forwardSlipSeconds, 0.06);
        assertTrue("still misses the coded end points: " + tuner.getLog(), tuner.isMatched());
    }

    @Test
    public void tunesATankDriveWithoutStrafing() {
        PurePursuitFollower.Params tuning = new PurePursuitFollower.Params();
        AutoTuner tuner = run(heavierRobot(), false, tuning);
        assertFalse(tuner.getStatus(), tuner.isAborted());
        assertTrue("still misses the coded end points: " + tuner.getLog(), tuner.isMatched());
    }

    @Test
    public void tunedValuesFollowTheTestRoutesOnTheSameRobot() {
        DriveSimulator.Params real = heavierRobot();
        PurePursuitFollower.Params tuning = new PurePursuitFollower.Params();
        run(real, true, tuning);
        RouteSimulation sim = new RouteSimulation(TestRoutes.STRAIGHT, true, tuning, real);
        sim.runToEnd(30);
        Pose2d end = sim.getPose();
        assertEquals(24, end.position.x, 1);
        assertEquals(-48, end.position.y, 1);
    }

    @Test
    public void stopsWhenTheRobotLeavesTheSafeArea() {
        Pose2d start = TestRoutes.TUNING_COURSE.startPose;
        AutoTuner tuner = new AutoTuner(start, new PurePursuitFollower.Params(), true);
        tuner.update(new Pose2d(0, 0, 0), new PoseVelocity2d(new com.acmerobotics.roadrunner.Vector2d(0, 0), 0), LOOP);
        assertTrue(tuner.isAborted());
    }
}
