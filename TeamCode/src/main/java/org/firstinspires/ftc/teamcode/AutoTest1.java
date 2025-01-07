package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Comp5218MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class AutoTest1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        Comp5218MecanumDrive drive = new Comp5218MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .lineToXConstantHeading(27)
                        .lineToYConstantHeading(-27)
                        .lineToXConstantHeading(47)
                        .lineToYConstantHeading(-37)
                        .lineToXConstantHeading(3)
                        .build());
    }
}
