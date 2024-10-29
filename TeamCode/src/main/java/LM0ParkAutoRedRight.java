
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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


@Autonomous(name = "LM0ParkAutoRedRight")
public class LM0ParkAutoRedRight extends Robot {

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
    private static final double CLAW_DOWN = 0.6;


    DeadReckonPath driveToParkPath;
    DeadReckonPath liftToSmallJunctionPath;




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
        driveToParkPath = new DeadReckonPath();
        liftToSmallJunctionPath = new DeadReckonPath();

        driveToParkPath.stop();
        liftToSmallJunctionPath.stop();


        //driveToParkPath
        driveToParkPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,5 , 0.25);

        //liftToSmallJunctionPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 8, 0.5);



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
      //  frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      //  frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
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
