package org.firstinspires.ftc.teamcode.purepursuit;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.File;
import java.io.FileWriter;

public class TuningFileTest {
    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private static TuningFile tuned() {
        PurePursuitFollower.Params follower = new PurePursuitFollower.Params();
        follower.forwardSlipSeconds = 0.217;
        follower.minTurnPower = 0.13;
        DriveSimulator.Params drive = new DriveSimulator.Params();
        drive.maxForwardVel = 63.6;
        drive.maxAngVel = Math.toRadians(308);
        return TuningFile.of(follower, drive, "Nov 7 4:12 PM");
    }

    @Test
    public void savesAndLoadsOnTheControlHub() throws Exception {
        File file = new File(folder.getRoot(), "FIRST/purepursuit_tuning.properties");
        tuned().save(file);

        TuningFile loaded = TuningFile.load(file);
        PurePursuitFollower.Params follower = new PurePursuitFollower.Params();
        loaded.applyTo(follower);
        assertEquals(0.217, follower.forwardSlipSeconds, 1e-4);
        assertEquals(0.13, follower.minTurnPower, 1e-4);
        assertEquals(new PurePursuitFollower.Params().decelDistance, follower.decelDistance, 1e-9);
        assertEquals("Nov 7 4:12 PM", loaded.savedAt);
    }

    @Test
    public void travelsInAViewerFrame() {
        ViewerFrame frame = new ViewerFrame();
        frame.tuning = tuned().encode();
        TuningFile received = TuningFile.decode(ViewerFrame.decode(frame.encode()).tuning);
        DriveSimulator.Params drive = new DriveSimulator.Params();
        received.applyTo(drive);
        assertEquals(63.6, drive.maxForwardVel, 1e-4);
        assertEquals(Math.toRadians(308), drive.maxAngVel, 1e-4);
        assertEquals("Nov 7 4:12 PM", received.savedAt);
    }

    @Test
    public void framesWithoutTuningStayThatWay() {
        assertNull(ViewerFrame.decode(new ViewerFrame().encode()).tuning);
    }

    @Test
    public void aMissingOrBrokenFileMeansDefaults() throws Exception {
        assertNull(TuningFile.load(new File(folder.getRoot(), "nope.properties")));
        File broken = folder.newFile("broken.properties");
        try (FileWriter w = new FileWriter(broken)) {
            w.write("follower.forwardSlipSeconds=oops\n");
        }
        assertNull(TuningFile.load(broken));
    }
}
