package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ContinuousServoTask;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import team25core.DeadReckonPath;
import team25core.DeadReckonTask;
import team25core.DistanceSensorTask;
import team25core.FourWheelDirectDrivetrain;
import team25core.OneWheelDirectDrivetrain;
import team25core.OneWheelDriveTask;
import team25core.Robot;
import team25core.RobotEvent;
import team25core.SingleShotTimerTask;
import team25core.TwoWheelDirectDrivetrain;
import team25core.vision.apriltags.TwoWheelDriveTask;

@Autonomous(name = "BlueAuto")
public class TerkelAuto extends Robot {

    private ElapsedTime timer;


    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor outtake;

    private TwoWheelDirectDrivetrain liftDriveTrain;
    private DcMotor liftMotor;
    private OneWheelDirectDrivetrain outtakeDrivetrain;


    private FourWheelDirectDrivetrain drivetrain;
//    private static final double GIZA_CLAW_LEFT_CLOSE = 0.1;
//    private static final double GIZA_CLAW_LEFT_OPEN = 0.6;
//
//    //gizaClawRightServo Positions
//    private static final double GIZA_CLAW_RIGHT_CLOSE = 0.9;
//    private static final double GIZA_CLAW_RIGHT_OPEN = 0.4;
//
//    private Servo gizaClawRightServo;
//    private Servo gizaClawLeftServo;

    private static final double WHEELIE_UP = 0.05;
    private static final double WHEELIE_DOWN = .95;

    private static final double MINI_CLAW_OPEN = 0.3;
    private static final double MINI_CLAW_CLOSE = 0.75;

    //groundMiniClawServo Positions
    private static final double GM_CLAW_OPEN = 0.3;
    private static final double GM_CLAW_CLOSE = 0.75;


    //armServo Positions
    private static final double ARM_FRONT = .07;
    private static final double ARM_BACK = .71;
    //armServo Positions


    public String position;
    private DeadReckonPath outtakePath;

    private CRServo horizontalLeftServo;
    private CRServo horizontalRightServo;

    private Servo horizontalLiftServo;
    private Servo armServo;
    private Servo groundMiniClawServo;


    private Servo wheelieServo;
    private Servo wheelieRotationServo;

    private Servo miniClawServo;
    private Servo gizaClawLeftServo;
    private Servo gizaClawRightServo;


    //private ContinuousTwoServoTask horizontalLiftTask;
    // private ContinuousServoTask horizontalRightLiftTask;

    private ContinuousServoTask horizontalRightLiftTask;
    private ContinuousServoTask horizontalLeftLiftTask;
    private TwoWheelDriveTask twoLiftTask;
    private DcMotor hangLeftMotor;
    private DcMotor hangRightMotor;
    private DcMotor liftRightMotor;
    private DcMotor liftLeftMotor;


    public static double LIFT_DISTANCE = 60;
    public static double LIFT_SPEED = .6;
    public static double LOWER_DISTANCE =35;
    public static double LOWER_LIFT_SPEED = -0.6;



    private Telemetry.Item locationTlm;
    private Telemetry.Item whereAmI;
    private Telemetry.Item eventTlm;

    private DeadReckonPath driveToObservationPath;

    private DeadReckonPath driveToSubmersiblePath;



    private DeadReckonPath lowerLiftPath;

    private DeadReckonPath liftToBoardPath;



    @Override
    public void handleEvent(RobotEvent e)
    {
        whereAmI.setValue("in handleEvent");
        /*
         * Every time we complete a segment drop a note in the robot log.
         */
        if (e instanceof DeadReckonTask.DeadReckonEvent) {
            RobotLog.i("Completed path segment %d", ((DeadReckonTask.DeadReckonEvent)e).segment_num);
        }
    }

    public void driveToSubmersible(DeadReckonPath driveToSubmersiblePath) {
        whereAmI.setValue("in driveToSignalZone");
        RobotLog.i("drives straight onto the launch line");
//        gizaClawLeftServo.setPosition(GIZA_CLAW_LEFT_OPEN);
//        gizaClawRightServo.setPosition(GIZA_CLAW_RIGHT_OPEN);
        delay(200);
        this.addTask(new DeadReckonTask(this, driveToSubmersiblePath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
//                gizaClawLeftServo.setPosition(GIZA_CLAW_LEFT_OPEN);
//                gizaClawRightServo.setPosition(GIZA_CLAW_RIGHT_OPEN);
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("infront of submersible");
                   // lowerLiftToPlaceSpecimen();

                }
            }
        });
    }

    public void driveToObservation(DeadReckonPath driveToObservationPath) {
        whereAmI.setValue("in driveToSignalZone");
        RobotLog.i("drives straight onto the launch line");

        this.addTask(new DeadReckonTask(this, driveToObservationPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {

                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("drive to observation");

                }
            }
        });
    }

    private void delay(int delayInMsec) {
        this.addTask(new SingleShotTimerTask(this, delayInMsec) {
            @Override
            public void handleEvent(RobotEvent e) {
                SingleShotTimerEvent event = (SingleShotTimerEvent) e;
                if (event.kind == EventKind.EXPIRED ) {
                    whereAmI.setValue("in delay task");

                }
            }
        });

    }



    public void liftToPlacePixelOnBoard() {
        this.addTask(new DeadReckonTask(this, liftToBoardPath, liftDriveTrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("liftedToBoard");
                    driveToSubmersible(driveToSubmersiblePath);
                }
            }
        });
    }
    public void lowerLiftToPlaceSpecimen() {
        this.addTask(new DeadReckonTask(this, lowerLiftPath, liftDriveTrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("placed specimen");
//                    gizaClawLeftServo.setPosition(GIZA_CLAW_LEFT_OPEN);
//                    gizaClawRightServo.setPosition(GIZA_CLAW_RIGHT_OPEN);
                    driveToObservation(driveToObservationPath);


                }
            }
        });
    }



    public void init()
    {
        groundMiniClawServo = hardwareMap.servo.get("groundMiniClawServo");
        armServo = hardwareMap.servo.get("armServo");
        miniClawServo = hardwareMap.servo.get("miniClawServo");
        wheelieRotationServo = hardwareMap.servo.get("wheelieRotationServo");
        // hardware mapping
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        gizaClawLeftServo = hardwareMap.servo.get("gizaClawLeftServo");
        gizaClawRightServo = hardwareMap.servo.get("gizaClawRightServo");

        horizontalLeftServo=hardwareMap.crservo.get("hzLeftServo");
        horizontalRightServo=hardwareMap.crservo.get("horizontalRightServo");

        hangLeftMotor = hardwareMap.get(DcMotor.class,"hangLeftMotor");
        hangLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armServo.setPosition(ARM_BACK);
        groundMiniClawServo.setPosition(GM_CLAW_CLOSE);
        miniClawServo.setPosition(MINI_CLAW_CLOSE);
        wheelieRotationServo.setPosition(WHEELIE_UP);
        hangRightMotor = hardwareMap.get(DcMotor.class,"hangRightMotor");
        hangRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        liftRightMotor = hardwareMap.get(DcMotor.class,"liftRightMotor");
        liftRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftLeftMotor = hardwareMap.get(DcMotor.class,"liftLeftMotor");
        liftLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftDriveTrain = new TwoWheelDirectDrivetrain(liftRightMotor,liftLeftMotor);

        // instantiating FourWheelDirectDrivetrain
        drivetrain = new FourWheelDirectDrivetrain(frontRight, backRight, frontLeft, backLeft);

        //sets motors position to 0
        drivetrain.resetEncoders();

//        gizaClawRightServo.setPosition(GIZA_CLAW_LEFT_CLOSE);
//        gizaClawLeftServo.setPosition(GIZA_CLAW_RIGHT_CLOSE);


        //motor will try to tun at the targeted velocity
        drivetrain.encodersOn();

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //left positive (backwards)
        horizontalLeftServo.setPower(0);
        // right negative (backwards)
        horizontalRightServo.setPower(0);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       // twoLiftTask=new TwoWheelDriveTask(this, liftLeftMotor,liftRightMotor,true);
        //telemetry


        // telemetry shown on the phone
        whereAmI = telemetry.addData("location in code", "init");


        initPaths();
    }

    public void start()
    {
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        whereAmI.setValue("in Start");
        liftToPlacePixelOnBoard();
    }

    public void initPaths() {


        driveToObservationPath = new DeadReckonPath();

        driveToSubmersiblePath = new DeadReckonPath();


        liftToBoardPath = new DeadReckonPath();
        liftToBoardPath.stop();
        liftToBoardPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, LIFT_DISTANCE, LIFT_SPEED);

        lowerLiftPath = new DeadReckonPath();
        lowerLiftPath.stop();
        lowerLiftPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, LOWER_DISTANCE, LOWER_LIFT_SPEED);




        driveToSubmersiblePath.stop();
        driveToObservationPath.stop();

        driveToSubmersiblePath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 13, 0.5);
        //driveToSubmersiblePath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 5, -0.25);
        driveToObservationPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 8, -0.25);

        driveToObservationPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 25, 0.25);

    }
}