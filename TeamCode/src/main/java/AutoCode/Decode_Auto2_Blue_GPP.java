package AutoCode;

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "Decode_Auto2_Blue_GPP", group = "Examples")
public class Decode_Auto2_Blue_GPP extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    //Declare your Servo
    private CRServo canopyServo;
    private Servo rliftServo;
    private Servo lliftServo;
    private Servo sorterServo;

    private DcMotor launcherMotor;
    private DcMotor intakeMotor;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(110)); // Start Pose of our robot.
    private final Pose scanPose = new Pose(72, 128, Math.toRadians(90)); // Scan Pose of our robot for april Tag. It is facing the AprilTag at a 90 degree angle.

    private final Pose scorePose = new Pose(56, 8, Math.toRadians(110)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private final Pose pickup1Pose = new Pose(42, 36, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose Collect1_1 = new Pose(35, 36, Math.toRadians(180));
    private final Pose Collect1_2 = new Pose(21, 60, Math.toRadians(180));
//    private final Pose Collect1_3 = new Pose(21, 60, Math.toRadians(180));
//    private final Pose Collect1_4 = new Pose(48, 60, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(42, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose Collect2_1 = new Pose(35, 36, Math.toRadians(180));
    private final Pose Collect2_2 = new Pose(30, 36, Math.toRadians(180));
    private final Pose Collect2_3 = new Pose(21, 36, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(42, 84, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose Collect3_1 = new Pose(30, 84, Math.toRadians(180));
    private final Pose Collect3_2 = new Pose(21, 84, Math.toRadians(180));
    private final Pose LeavePose = new Pose(35, 8, Math.toRadians(180)); // Leave Pose at the end of Auto.

    private Path scorePreload;
//    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, Leave;
    private PathChain grabPickup1, CollectSet1_1, CollectSet1_2, scorePickup1, grabPickup2, CollectSet2_1, CollectSet2_2, CollectSet2_3, scorePickup2, grabPickup3, CollectSet3_1, CollectSet3_2, scorePickup3, Leave;


public void buildPaths() {
    /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
//    scanTag = new Path(new BezierLine(startPose, scanPose));
//        scanTag.setLinearHeadingInterpolation(startPose.getHeading(), scanPose.getHeading());

//    scorePreload = new Path(new BezierLine(startPose, scorePose));
//    scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
//    /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//    scorePreload1 = follower.pathBuilder()
//            .addPath(new BezierLine(scorePose, scorePose))
//            .setLinearHeadingInterpolation(scorePose.getHeading(), scorePose.getHeading())
////            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//            .build();

    scorePreload = new Path(new BezierLine(startPose, scorePose));
    scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
    /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */

    grabPickup1 = follower.pathBuilder()
            .addPath(new BezierLine(scorePose, pickup1Pose))
            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    CollectSet1_1 = follower.pathBuilder()
            .addPath(new BezierLine(pickup1Pose, Collect1_1))
            .setLinearHeadingInterpolation(pickup1Pose.getHeading(), Collect1_1.getHeading())
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();
    CollectSet1_2 = follower.pathBuilder()
            .addPath(new BezierLine(Collect1_1, Collect1_2))
            .setLinearHeadingInterpolation(Collect1_1.getHeading(), Collect1_2.getHeading())
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();
    /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
    scorePickup1 = follower.pathBuilder()
            .addPath(new BezierLine(Collect1_2, scorePose))
                .setLinearHeadingInterpolation(Collect1_2.getHeading(), scorePose.getHeading())
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

   /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
    grabPickup2 = follower.pathBuilder()
            .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    CollectSet2_1 = follower.pathBuilder()
            .addPath(new BezierLine(pickup2Pose, Collect2_1))
            .setLinearHeadingInterpolation(pickup2Pose.getHeading(), Collect2_1.getHeading())
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();
    CollectSet2_2 = follower.pathBuilder()
            .addPath(new BezierLine(Collect2_1, Collect2_2))
            .setLinearHeadingInterpolation(Collect2_1.getHeading(), Collect2_2.getHeading())
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();
    CollectSet2_3 = follower.pathBuilder()
            .addPath(new BezierLine(Collect2_2, Collect2_3))
            .setLinearHeadingInterpolation(Collect2_2.getHeading(), Collect2_3.getHeading())
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();
    /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
    scorePickup2 = follower.pathBuilder()
            .addPath(new BezierLine(Collect2_3, scorePose))
                .setLinearHeadingInterpolation(Collect2_3.getHeading(), scorePose.getHeading())
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
    grabPickup3 = follower.pathBuilder()
            .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();
    CollectSet3_1 = follower.pathBuilder()
            .addPath(new BezierLine(pickup3Pose, Collect3_1))
            .setLinearHeadingInterpolation(pickup3Pose.getHeading(), Collect3_1.getHeading())
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();
    CollectSet3_2 = follower.pathBuilder()
            .addPath(new BezierLine(Collect3_1, Collect3_2))
            .setLinearHeadingInterpolation(Collect3_1.getHeading(), Collect3_2.getHeading())
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();
    /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
    scorePickup3 = follower.pathBuilder()
            .addPath(new BezierLine(Collect3_2, scorePose))
                .setLinearHeadingInterpolation(Collect3_2.getHeading(), scorePose.getHeading())
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    Leave = follower.pathBuilder()
            .addPath(new BezierLine(scorePose, LeavePose))
            .setLinearHeadingInterpolation(scorePose.getHeading(), LeavePose.getHeading())
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();
}

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                launcherMotor.setPower(-0.92);
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                canopyServo.setPower(-40);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                /* Score Preload */
                rliftServo.setPosition(0.24);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                rliftServo.setPosition(0.05);
                launcherMotor.setPower(-0.90);
                try {
                    sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                lliftServo.setPosition(0.01);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                lliftServo.setPosition(0.2);
                launcherMotor.setPower(-0.90);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                lliftServo.setPosition(0.01);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                lliftServo.setPosition(0.2);

                setPathState(2);
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
//               if (!follower.isBusy()) {
                follower.followPath(grabPickup1, true);
                setPathState(3);
//                }
                break;
            case 3:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if (!follower.isBusy()) {
                    /*Start Intake Wheel*/

                    //intakeServo.setPower(-100);
                    intakeMotor.setPower(-1);
                    canopyServo.setPower(-40);
                    sorterServo.setPosition(0.62); // sorter Left to grab green balls

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(CollectSet1_1, true);
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    sorterServo.setPosition(0.15); // sorter right to grab purple ball
                    follower.followPath(CollectSet1_2, true);
                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    intakeMotor.setPower(0);
                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    setPathState(4);
//                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
//                if (!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                follower.followPath(scorePickup1, true);
                launcherMotor.setPower(-0.92);
                canopyServo.setPower(-40);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                /* Score Preload */
                rliftServo.setPosition(0.24);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                rliftServo.setPosition(0.05);
                launcherMotor.setPower(-0.90);
                try {
                    sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                lliftServo.setPosition(0.01);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                lliftServo.setPosition(0.2);
                launcherMotor.setPower(-0.90);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                lliftServo.setPosition(0.01);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                lliftServo.setPosition(0.2);
                    /* Score First Set End*/

                    setPathState(5);
//                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if (!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2, true);
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    setPathState(6);
//                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
//                if (!follower.isBusy()) {
                    //intakeServo.setPower(100);
                    intakeMotor.setPower(-1);
                    canopyServo.setPower(-40);
                    sorterServo.setPosition(0.15);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(CollectSet2_1, true);
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    sorterServo.setPosition(0.62);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(CollectSet2_2, true);
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    sorterServo.setPosition(0.15);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(CollectSet2_3, true);
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    //intakeServo.setPower(0);
                    intakeMotor.setPower(0);
                    setPathState(7);
//                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if (!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scorePickup2, true);
                launcherMotor.setPower(-0.92);
                canopyServo.setPower(-40);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                rliftServo.setPosition(0.24);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                rliftServo.setPosition(0.05);
                launcherMotor.setPower(-0.90);
                try {
                    sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                lliftServo.setPosition(0.01);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                lliftServo.setPosition(0.2);
                launcherMotor.setPower(-0.90);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                lliftServo.setPosition(0.01);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                lliftServo.setPosition(0.2);
                    /* Score Second Set End*/
                    setPathState(8);
//                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
//                if (!follower.isBusy()) {


                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(grabPickup3, true);
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    setPathState(9);
//                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
//                if (!follower.isBusy()) {
                        //intakeServo.setPower(100);
                        intakeMotor.setPower(-1);
                        canopyServo.setPower(-40);
                        sorterServo.setPosition(0.15);
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(CollectSet3_1, true);
                        try {
                            sleep(1000);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                    sorterServo.setPosition(0.62);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(CollectSet3_2, true);
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }

                        //intakeServo.setPower(0);
                        intakeMotor.setPower(0);
                    setPathState(10);
//                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if (!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scorePickup3, true);
                launcherMotor.setPower(-0.92);
                canopyServo.setPower(-40);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                /* Score Preload */
                rliftServo.setPosition(0.24);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                rliftServo.setPosition(0.05);
                launcherMotor.setPower(-0.90);
                try {
                    sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                lliftServo.setPosition(0.01);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                lliftServo.setPosition(0.2);
                launcherMotor.setPower(-0.90);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                lliftServo.setPosition(0.01);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                lliftServo.setPosition(0.2);
                    /* Score Third Set End*/

                        setPathState(11);

//                }
                break;

            case 11:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
//                if (!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(Leave, true);
                    setPathState(12);
//                }
                break;
            case 12:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    launcherMotor.setPower(0);
                    canopyServo.setPower(0);
                    intakeMotor.setPower(0);
                    setPathState(-1);
//                }

                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();



        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        //intakeServo = hardwareMap.get(CRServo.class,"AxonServo");
        canopyServo = hardwareMap.get(CRServo.class, "Canopy");
        rliftServo = hardwareMap.get(Servo.class, "Rlift");
        lliftServo = hardwareMap.get(Servo.class, "Llift");
        sorterServo = hardwareMap.get(Servo.class, "Sorter");
        launcherMotor = hardwareMap.dcMotor.get("launcher");
        intakeMotor = hardwareMap.dcMotor.get("grabber");




        //colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

     //   cd.init(hardwareMap);

/******** Setting ZeroPowerBehaviour *******/
//        DcMotorEx fL = hardwareMap.dcMotor.get("front_left_drive");
//        DcMotorEx fR = hardwareMap.dcMotor.get("front_right_drive");
//        DcMotorEx bL = hardwareMap.dcMotor.get("back_left_drive");
//        DcMotorEx bR = hardwareMap.dcMotor.get("back_right_drive");
//        allDriveMotors = Arrays.asList(fL, fR, bL, bR); // Example motor list
//        configureMotors(); // Apply zero power behavior to all motors


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();

         setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}