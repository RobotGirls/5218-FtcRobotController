package org.firstinspires.ftc.teamcode.opmodes;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import team25core.DeadReckonPath;
import team25core.DeadReckonTask;
import team25core.DistanceSensorTask;
import team25core.FourWheelDirectDrivetrain;
import team25core.OneWheelDirectDrivetrain;
import team25core.Robot;
import team25core.RobotEvent;
import team25core.SingleShotTimerTask;
@Disabled
@Autonomous(name = "BucketRedAuto")
public class BucketRedAuto extends Robot {

    private ElapsedTime timer;


    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor outtake;

    private OneWheelDirectDrivetrain liftMotorDrivetrain;
    private DcMotor liftMotor;
    private OneWheelDirectDrivetrain outtakeDrivetrain;


    private FourWheelDirectDrivetrain drivetrain;


    //gizaClawLeftServo Positions
    private static final double GIZA_CLAW_LEFT_OPEN = 0.3;
    private static final double GIZA_CLAW_LEFT_CLOSE = 0.7;

    //gizaClawRightServo Positions
    private static final double GIZA_CLAW_RIGHT_OPEN = 0.7;
    private static final double GIZA_CLAW_RIGHT_CLOSE = 0.1;
    //wheelieServo Positions
    private static final double WHEELIE_GRAB = 0.1;
    private static final double WHEELIE_RELEASE = 0.99;

    private Servo gizaClawRightServo;
    private Servo gizaClawLeftServo;
    private Servo wheelieServo;

    public String position;
    private DeadReckonPath outtakePath;



    public static double LIFT_DISTANCE = 125;
    public static double LIFT_SPEED = .6;
    public static double LOWER_DISTANCE = 80;
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
        //lowerLiftToPlaceSpecimen();
        whereAmI.setValue("in driveToSignalZone");
        RobotLog.i("drives straight onto the launch line");
        //delay(200);
        lowerLiftToPlaceSpecimen();
        this.addTask(new DeadReckonTask(this, driveToSubmersiblePath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("infront of submersible");
                    lowerLiftToPlaceSpecimen();
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
                    delay(300);
                    wheelieServo.setPosition(WHEELIE_RELEASE);
                   // gizaClawRightServo.setPosition(GIZA_CLAW_RIGHT_OPEN);
                    delay(200);
                    driveToSubmersible(driveToSubmersiblePath);
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
        this.addTask(new DeadReckonTask(this, liftToBoardPath, liftMotorDrivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("liftedToBoard");
                    driveToObservation(driveToObservationPath);
                }
            }
        });
    }
    public void lowerLiftToPlaceSpecimen() {
        this.addTask(new DeadReckonTask(this, lowerLiftPath, liftMotorDrivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("placed specimen");
//                    gizaClawLeftServo.setPosition(GIZA_CLAW_LEFT_OPEN);
//                    gizaClawRightServo.setPosition(GIZA_CLAW_RIGHT_OPEN);
                    //delay(200);
                    // driveToObservation(driveToObservationPath);


                }
            }
        });
    }



    public void init()
    {
        // hardware mapping
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        gizaClawLeftServo = hardwareMap.servo.get("gizaClawLeftServo");
        gizaClawRightServo = hardwareMap.servo.get("gizaClawRightServo");



        // instantiating FourWheelDirectDrivetrain
        drivetrain = new FourWheelDirectDrivetrain(frontRight, backRight, frontLeft, backLeft);

        //sets motors position to 0
        drivetrain.resetEncoders();

//        gizaClawRightServo.setPosition(GIZA_CLAW_LEFT_CLOSE);
//        gizaClawLeftServo.setPosition(GIZA_CLAW_RIGHT_CLOSE);
//     gizaClawRightServo.setPosition(GIZA_CLAW_LEFT_CLOSE);
       wheelieServo.setPosition(WHEELIE_GRAB);


        //motor will try to tun at the targeted velocity
        drivetrain.encodersOn();

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorDrivetrain = new OneWheelDirectDrivetrain(liftMotor);
        liftMotorDrivetrain.resetEncoders();
        liftMotorDrivetrain.encodersOn();


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

        driveToObservationPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 2, 0.25);
        //driveToSubmersiblePath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 5, -0.25);
        driveToObservationPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 10, 0.1);
        driveToObservationPath.addSegment(DeadReckonPath.SegmentType.TURN, 10, -0.1);
        driveToObservationPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.1);


        driveToSubmersiblePath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 8, -0.25);
        driveToSubmersiblePath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 25, 0.25);
        driveToSubmersiblePath.addSegment(DeadReckonPath.SegmentType.TURN, 70, -0.25);

        driveToSubmersiblePath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 18, 0.25);


    }
}

