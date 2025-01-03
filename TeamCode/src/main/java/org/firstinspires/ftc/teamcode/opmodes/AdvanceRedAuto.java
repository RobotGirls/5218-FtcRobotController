package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import team25core.TwoWheelDirectDrivetrain;

@Autonomous(name = "AUTO")
public class AdvanceRedAuto extends Robot {

    private ElapsedTime timer;


    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor hangLeftMotor;
    private DcMotor hangRightMotor;


    private DcMotor outtake;

    private OneWheelDirectDrivetrain liftMotorDrivetrain;
    private DcMotor liftMotor;
    private OneWheelDirectDrivetrain outtakeDrivetrain;


    private FourWheelDirectDrivetrain drivetrain;
    private DeadReckonPath  driveToParkPath;
    private DeadReckonPath hangLiftPath;
    private TwoWheelDirectDrivetrain hangLiftDriveTrain;
    private static final double MINI_CLAW_OPEN = .05;
    private static final double MINI_CLAW_CLOSE = 0.8;


    private Servo miniClawServo;

    //gizaClawLeftServo Positions

    private static final double GIZA_CLAW_LEFT_OPEN = 0.25;
    private static final double GIZA_CLAW_LEFT_CLOSE = 0.9;

    //gizaClawRightServo Positions
    private static final double GIZA_CLAW_RIGHT_OPEN = 0.65;
    private static final double GIZA_CLAW_RIGHT_CLOSE = 0.1;

    private Servo gizaClawRightServo;
    private Servo gizaClawLeftServo;

    public static double HANG_LIFT_DISTANCE = 30;
    public static double HANG_LIFT_SPEED =1;
    public String position;
    private DeadReckonPath outtakePath;



    public static double LIFT_DISTANCE = 55;
    public static double LIFT_SPEED = .95;

    public static double LIFT_OFF_WALL_DISTANCE = 60;
    public static double LIFT_OFF_WALL_SPEED = .6;


    public static double LOWER_DISTANCE = 10;
    public static double LOWER_LIFT_SPEED = -0.6;



    private Telemetry.Item locationTlm;
    private Telemetry.Item whereAmI;
    private Telemetry.Item eventTlm;

    private DeadReckonPath driveToSamplePath;


    private DeadReckonPath driveToObservationPath;

    private DeadReckonPath driveToSubmersiblePath;

    private DeadReckonPath driveToSpecimenPath;

    private DeadReckonPath lowerLiftPath;

    private DeadReckonPath liftToBoardPath;

    private DeadReckonPath liftOffTheWallPath;
    private DeadReckonPath lowerLiftToPlaceSpecimenTwoPath;


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
        delay(200);
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
    public void driveToSample(DeadReckonPath driveToSamplePath) {

        whereAmI.setValue("in driveToSignalZone");
        RobotLog.i("drives straight onto the launch line");
        delay(200);
        this.addTask(new DeadReckonTask(this, driveToSamplePath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    driveToPark(driveToParkPath);
                    RobotLog.i("infront of submersible");

                    delay(100);
                   // liftOffTheWall();



                }
            }
        });
    }
    public void driveToPark(DeadReckonPath driveToParkPath) {
        gizaClawLeftServo.setPosition(GIZA_CLAW_LEFT_CLOSE);
        gizaClawRightServo.setPosition(GIZA_CLAW_RIGHT_CLOSE);
        whereAmI.setValue("in driveToSignalZone");
        RobotLog.i("drives straight onto the launch line");
        this.addTask(new DeadReckonTask(this, driveToParkPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {

                    RobotLog.i("infront of submersible");




                }
            }
        });
    }
    public void driveAwayFromSpecimen(DeadReckonPath driveToSpecimenPath) {
        whereAmI.setValue("in driveToSignalZone");
        RobotLog.i("drives straight onto the launch line");
        this.addTask(new DeadReckonTask(this, driveToSpecimenPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("drive To Specimen");
                    lowerLiftToPlaceSpecimenTwo();

                }
            }
        });
    }
    public void hangLiftUp() {
        this.addTask(new DeadReckonTask(this,  hangLiftPath, hangLiftDriveTrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                RobotLog.i(" hangLiftUp");
                if (path.kind == EventKind.PATH_DONE) {
                    liftToPlacePixelOnBoard();
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
        this.addTask(new DeadReckonTask(this, liftToBoardPath, liftMotorDrivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;

                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("liftedToBoard");
                    //driveToObservation(driveToObservationPath);
                }
            }
        });
    }
    public void liftOffTheWall() {
        gizaClawLeftServo.setPosition(GIZA_CLAW_LEFT_CLOSE);
        gizaClawRightServo.setPosition(GIZA_CLAW_RIGHT_CLOSE);
        this.addTask(new DeadReckonTask(this,  liftOffTheWallPath, liftMotorDrivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("liftedToBoard");
                    driveAwayFromSpecimen(driveToSpecimenPath);
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
                    delay(200);
                    driveToSample(driveToSamplePath);


                }
            }
        });
    }
    public void lowerLiftToPlaceSpecimenTwo() {
        this.addTask(new DeadReckonTask(this, lowerLiftToPlaceSpecimenTwoPath, liftMotorDrivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("placed specimen");
                    gizaClawLeftServo.setPosition(GIZA_CLAW_LEFT_CLOSE);
                    gizaClawRightServo.setPosition(GIZA_CLAW_RIGHT_CLOSE);
                    delay(150);
                    driveToObservation(driveToObservationPath);


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
        miniClawServo = hardwareMap.servo.get("miniClawServo");



        // instantiating FourWheelDirectDrivetrain
        drivetrain = new FourWheelDirectDrivetrain(frontRight, backRight, frontLeft, backLeft);

        //sets motors position to 0
        drivetrain.resetEncoders();

        gizaClawRightServo.setPosition(GIZA_CLAW_LEFT_CLOSE);
        gizaClawLeftServo.setPosition(GIZA_CLAW_RIGHT_CLOSE);


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
      // frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorDrivetrain = new OneWheelDirectDrivetrain(liftMotor);
        liftMotorDrivetrain.resetEncoders();
        liftMotorDrivetrain.encodersOn();
        hangRightMotor = hardwareMap.get(DcMotor.class, "hangRightMotor");
        hangRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        hangLeftMotor = hardwareMap.get(DcMotor.class, "hangLeftMotor");
        hangLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        hangLiftDriveTrain= new TwoWheelDirectDrivetrain(hangLeftMotor, hangRightMotor);
        hangLiftDriveTrain.resetEncoders();
        hangLiftDriveTrain.encodersOn();



        // telemetry shown on the phone
        whereAmI = telemetry.addData("location in code", "init");


        initPaths();
    }

    public void start()
    {
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        whereAmI.setValue("in Start");
        hangLiftUp();
    }

    public void initPaths() {

        hangLiftPath= new DeadReckonPath();
        hangLiftPath.stop();
        hangLiftPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT,HANG_LIFT_DISTANCE, HANG_LIFT_SPEED);

        driveToObservationPath = new DeadReckonPath();

        driveToSubmersiblePath = new DeadReckonPath();

        driveToSamplePath = new DeadReckonPath();
        driveToSamplePath.stop();

        driveToSpecimenPath = new DeadReckonPath();
        driveToSpecimenPath.stop();

        liftToBoardPath = new DeadReckonPath();
        liftToBoardPath.stop();
        liftToBoardPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, LIFT_DISTANCE, LIFT_SPEED);

        liftOffTheWallPath= new DeadReckonPath();
        liftOffTheWallPath.stop();
        liftOffTheWallPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, LIFT_OFF_WALL_DISTANCE, LIFT_OFF_WALL_SPEED);

        lowerLiftToPlaceSpecimenTwoPath= new DeadReckonPath();
        lowerLiftToPlaceSpecimenTwoPath.stop();
        lowerLiftToPlaceSpecimenTwoPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT,LOWER_DISTANCE, LOWER_LIFT_SPEED);

        lowerLiftPath = new DeadReckonPath();
        lowerLiftPath.stop();
        lowerLiftPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, LOWER_DISTANCE, LOWER_LIFT_SPEED);


        driveToParkPath = new DeadReckonPath();
        driveToParkPath.stop();


        driveToSubmersiblePath.stop();
        driveToObservationPath.stop();

        driveToSubmersiblePath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 14, 0.2);
        //driveToSubmersiblePath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 5, -0.25);

        //driveToSamplePath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, -0.75);

        driveToSamplePath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, -0.6);


        driveToParkPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 14, 0.45);
        driveToParkPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 10, 0.75);
        driveToParkPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 4, 0.55);
        driveToParkPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 19, -0.45);
//        driveToSamplePath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 5, 0.35);
//        driveToSamplePath.addSegment(DeadReckonPath.SegmentType.TURN, 79, 0.5);
//        driveToSamplePath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 5, 0.75);



        driveToSpecimenPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 8, -0.75);
        driveToSpecimenPath.addSegment(DeadReckonPath.SegmentType.TURN, 81, 0.5);
        driveToSpecimenPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 22, -0.75);
      //  driveToSpecimenPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);



        driveToObservationPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 10, -0.25);

        driveToObservationPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 25, 0.25);

    }
}
