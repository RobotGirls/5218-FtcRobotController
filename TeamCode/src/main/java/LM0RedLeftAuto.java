
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import team25core.DeadReckonPath;
import team25core.DeadReckonTask;
import team25core.FourWheelDirectDrivetrain;
import team25core.OneWheelDirectDrivetrain;
import team25core.Robot;
import team25core.RobotEvent;
import team25core.SingleShotTimerTask;


@Autonomous(name = "LM0RedLeftAuto")
@Disabled
public class LM0RedLeftAuto extends Robot {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;


    private FourWheelDirectDrivetrain drivetrain;

    private OneWheelDirectDrivetrain liftMotorDrivetrain;
    private DcMotor liftMotor;

    private Servo marshClawServo;
    private Servo marshClawRotationServo;

    private static final double CLAW_GRAB = 0.99;
    private static final double CLAW_RELEASE = 0.6;
    private static final double CLAW_UP = 0.35;
    private static final double LIFT_DISTANCE = 12;
    private static final double LIFT_SPEED = 0.35;

    DeadReckonPath driveToParkPath;
    DeadReckonPath driveToBasketPath;
    DeadReckonPath liftToPlaceSpecimenInBasketPath;




    private Telemetry.Item whereAmI;

    /*
     * The default event handler for the robot.
     */

    @Override
    public void handleEvent(RobotEvent e)
    {
        /*
         * Every time we complete a segment drop a note in the robot log.
         */
        if (e instanceof DeadReckonTask.DeadReckonEvent) {
            RobotLog.i("Completed path segment %d", ((DeadReckonTask.DeadReckonEvent)e).segment_num);
        }
    }


    private void driveToBaskets(DeadReckonPath driveToBasketPath)
    {
        whereAmI.setValue("in driveToBasketPath");
        RobotLog.i("drives to Basket");

        this.addTask(new DeadReckonTask(this, driveToBasketPath, drivetrain){
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("finished driving to Basket");
                    liftToPlaceSpecimenInBasket();

                }
            }
        });
    }
    private void driveToPark(DeadReckonPath driveToParkPath)
    {
        whereAmI.setValue("in driveToPark");
        RobotLog.i("drives to First Ground Junction");

        this.addTask(new DeadReckonTask(this, driveToParkPath, drivetrain){
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("finished parking");
                }
            }
        });
    }
    public void liftToPlaceSpecimenInBasket() {
        this.addTask(new DeadReckonTask(this, liftToPlaceSpecimenInBasketPath, liftMotorDrivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("lifted to basket");
                    marshClawRotationServo.setPosition(CLAW_RELEASE);
                    driveToPark(driveToParkPath);

//
                }
            }
        });
    }


    public void delay(int seconds)
    {
        this.addTask(new SingleShotTimerTask(this, 1000*seconds) {
            @Override
            public void handleEvent (RobotEvent e){
                SingleShotTimerEvent event = (SingleShotTimerEvent) e;
                switch(event.kind) {
                    case EXPIRED:
                        break;
                }
            }
        });
    }



    public void initPaths()
    {
        liftToPlaceSpecimenInBasketPath = new DeadReckonPath();
        liftToPlaceSpecimenInBasketPath.stop();
        liftToPlaceSpecimenInBasketPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, LIFT_DISTANCE, LIFT_SPEED);
        driveToParkPath = new DeadReckonPath();
        driveToBasketPath = new DeadReckonPath();


        driveToParkPath.stop();
        driveToBasketPath.stop();

        driveToBasketPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT,3 , 0.5);
        driveToBasketPath.addSegment(DeadReckonPath.SegmentType.TURN,37.5 , 0.5);
        driveToBasketPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT,10 , 0.5);
        driveToBasketPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT,1 , 0.5);


        //driveToParkPath
        driveToParkPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,3 , -0.25);




    }

    @Override
    public void init()
    {
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");

        marshClawRotationServo = hardwareMap.servo.get("marshClawRotationServo");
        marshClawServo = hardwareMap.servo.get("marshClawServo");

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drivetrain = new FourWheelDirectDrivetrain(frontRight, backRight, frontLeft, backLeft);
        drivetrain.resetEncoders();
        drivetrain.encodersOn();

        liftMotorDrivetrain = new OneWheelDirectDrivetrain(liftMotor);
        liftMotorDrivetrain.resetEncoders();
        liftMotorDrivetrain.encodersOn();

        marshClawServo.setPosition(CLAW_GRAB);
        marshClawRotationServo.setPosition(CLAW_UP);

        whereAmI = telemetry.addData("location in code", "init");
        initPaths();

    }

    @Override
    public void start()
    {
        driveToPark(driveToParkPath);
        whereAmI.setValue("in Start");
    }
}
