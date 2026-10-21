
package org.firstinspires.ftc.teamcode.biobuzzautos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TankDrive;

import java.lang.Math;

@Autonomous(name = "BLUE Garden Park + Shooter")
public class integrateMechAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Starting position and heading
        Pose2d initialPose =
                new Pose2d(-66, 34, Math.toRadians(0));

        TankDrive drive = new TankDrive(hardwareMap, initialPose);

        // Existing parking trajectory (unchanged)
        TrajectoryActionBuilder toPark =
                drive.actionBuilder(initialPose)
                        .lineToX(34)
                        .turn(Math.toRadians(-90))
                        .lineToY(60);

        Action parkTrajectory = toPark.build();

        // Mechanism hardware
        DcMotorEx launcher =
                hardwareMap.get(DcMotorEx.class, "launcher");

        DcMotor intake =
                hardwareMap.get(DcMotor.class, "intake");

        CRServo windmill =
                hardwareMap.get(CRServo.class, "windmill");

        // Match the flywheel setup used in TeleOp
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setVelocityPIDFCoefficients(40, 0, 0, 12.5);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Park + Shooter Auto initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        try {
            // STEP 1: Complete the existing parking path
            Actions.runBlocking(
                    new SequentialAction(parkTrajectory)
            );

            if (!opModeIsActive()) {
                return;
            }

            // STEP 2: Spin up the flywheel
            launcher.setVelocity(1250);

            telemetry.addLine("Spinning up launcher...");
            telemetry.update();

            // Wait until the launcher reaches the target threshold
            while (opModeIsActive()
                    && launcher.getVelocity() < 1200) {

                telemetry.addData(
                        "Launcher velocity",
                        "%.0f", launcher.getVelocity()
                );
                telemetry.addData("Target velocity", 1250);
                telemetry.update();

                idle();
            }

            if (!opModeIsActive()) {
                return;
            }

            // STEP 3: Feed artifacts while the launcher spins
            // Adjust directions/power to match your tested TeleOp.
            intake.setPower(1.0);
            windmill.setPower(1.0);

            ElapsedTime feedTimer = new ElapsedTime();

            while (opModeIsActive()
                    && feedTimer.seconds() < 2.0) {

                telemetry.addData(
                        "Launcher velocity",
                        "%.0f", launcher.getVelocity()
                );
                telemetry.addLine("Feeding artifacts");
                telemetry.update();

                idle();
            }

        } finally {
            // Stop mechanisms even if the OpMode is interrupted
            launcher.setVelocity(0);
            intake.setPower(0);
            windmill.setPower(0);
        }
    }
}
