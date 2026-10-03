
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.opmodes.Limelight3ASensor;

@TeleOp(name = "LimelightDistance")

public class LimeLightDistance extends LinearOpMode {

    private double rotationPower;
    private final double BLOCK_NOTHING = 0.25;
    private final double BLOCK_BOTH = 0.05;

    private boolean useAutoAlign = false;


    double y;
    double x;
    double rx;
    //DcMotorEx IntakeMotor;
    ElapsedTime timer;

    DcMotorEx leftFront, leftBack, rightBack, rightFront;

 //   DcMotorEx FlywheelMotor;

    private double adjustedFlywheelPower;
    private boolean firstTime;

    //DcMotorEx TransportMotor;
    private Limelight3ASensor limelightSensor = new Limelight3ASensor();

    @Override
    public void runOpMode() throws InterruptedException {

        boolean intakeOn = false;
        boolean outtakeOn = false;

        initHardware();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            x = gamepad1.left_stick_x ; // Counteract imperfect strafing
            rx = gamepad1.right_stick_x;

            checkGamepad2();

            if (useAutoAlign) {

                limelightSensor.limelightProcessing(telemetry);
                alignToTag();
                //not done yet

            }

            double frontLeftPower = (y + x + rx) ;
            double backLeftPower = (y - x + rx) ;
            double frontRightPower = (y - x - rx) ;
            double backRightPower = (y + x - rx) ;

            telemetry.update();

        }

        limelightSensor.stopLimelightProcessing();
    }

    public void checkGamepad2() {
        if (gamepad2.a) {
            if (useAutoAlign) {
                useAutoAlign = false;
            } else {
                useAutoAlign = true;
            }
        }

    }
    public void initHardware() {

//        FlywheelMotor = hardwareMap.get(DcMotorEx.class, "FlywheelMotor");
//        FlywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        FlywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        timer = new ElapsedTime();
        limelightSensor.initLimelight(hardwareMap, telemetry);
        telemetry.update();
        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftBack = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightBack = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // FIXME cindy need to figue out if this is run without or run with encoder
        // note: you must set this after stop and reset encoder; otherwise, the robot won't move
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);



    }


    public void alignToTag(){

        rx = limelightSensor.getWheelPower();


    }
}



