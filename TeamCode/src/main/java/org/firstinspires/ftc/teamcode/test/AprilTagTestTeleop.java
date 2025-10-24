//package org.firstinspires.ftc.teamcode.test;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//@TeleOp(name = "JasmineAprilTagTestTeleop3")
//
//public class AprilTagTestTeleop extends LinearOpMode {
//
//    private JasmineAprilTag jasmineAprilTag = new JasmineAprilTag();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        initHardware();
//        waitForStart();
//        while (opModeIsActive()) {
//            jasmineAprilTag.telemetryAprilTag(telemetry);
//            jasmineAprilTag.cameraStreaming(gamepad1);
//
//            // Share the CPU.
//            sleep(20);
//        }
//        jasmineAprilTag.stopAprilTagProcessing();
//    }
//
//    public void initHardware() {
//        jasmineAprilTag.initAprilTag(hardwareMap, telemetry);
//    }
//
//}
