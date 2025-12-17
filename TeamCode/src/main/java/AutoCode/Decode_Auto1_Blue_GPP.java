package AutoCode;

import static java.lang.Thread.sleep;
import java.util.List;
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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//Color sensor libraries
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import android.graphics.Color;

import java.util.Arrays;


@Autonomous(name = "Decode_Auto1_Blue_GPP", group = "Examples")
public class Decode_Auto1_Blue_GPP extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    //Declare your Servo
//    private CRServo intakeServo;
    private CRServo canopyServo;
    private Servo rliftServo;
    private Servo lliftServo;
    private Servo sorterServo;

    private DcMotor launcherMotor;
    private DcMotor intakeMotor;
    //private NormalizedColorSensor colorSensor;

    private final float GREEN_HUE_MIN = 80;
    private final float GREEN_HUE_MAX = 145;

    private final Pose startPose = new Pose(28.5, 128, Math.toRadians(135)); // Start Pose of our robot.
    private final Pose scanPose = new Pose(72, 128, Math.toRadians(90)); // Scan Pose of our robot for april Tag. It is facing the AprilTag at a 90 degree angle.
    private final Pose scorePose = new Pose(60, 84, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(44, 84, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose Collect1_1 = new Pose(30, 84, Math.toRadians(180));
    private final Pose Collect1_2 = new Pose(24, 84, Math.toRadians(180));
    private final Pose Collect1_3 = new Pose(30, 74, Math.toRadians(90));
    private final Pose Collect1_4 = new Pose(20, 74, Math.toRadians(90));
    private final Pose pickup2Pose = new Pose(44, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose Collect2_1 = new Pose(36, 60, Math.toRadians(180));
    private final Pose Collect2_2 = new Pose(30, 60, Math.toRadians(180));
    private final Pose Collect2_3 = new Pose(24, 60, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(44, 36, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose Collect3_1 = new Pose(36, 36, Math.toRadians(180));
    private final Pose Collect3_2 = new Pose(24, 36, Math.toRadians(180));
    private final Pose LeavePose = new Pose(44, 84, Math.toRadians(90)); // Leave Pose at the end of Auto.

//    private Path scanTag;
    private Path scorePreload;
    //private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, Leave;
    private PathChain grabPickup1, CollectSet1_1, CollectSet1_2, CollectSet1_3, CollectSet1_4, scorePickup1, grabPickup2, CollectSet2_1, CollectSet2_2, CollectSet2_3, scorePickup2, grabPickup3, CollectSet3_1, CollectSet3_2, scorePickup3, Leave;

/*
    CRServo Intake = hardwareMap.get(CRServo.class, "AxonServo");
    CRServo Canopy = hardwareMap.get(CRServo.class, "Canopy");
    Servo Rlift = hardwareMap.get(Servo.class, "Rlift");
    Servo Llift = hardwareMap.get(Servo.class, "Llift");
    Servo Sorter = hardwareMap.get(Servo.class, "Sorter");
    DcMotor launcher = hardwareMap.dcMotor.get("launcher");
    NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
*/

//    ColorDetection cd = new ColorDetection();
//    ColorDetection.DetectedColor detectedColor;
/*******Setting ZeroPowerBehaviour******/
//
//    private DcMotorEx fL;
//    private DcMotorEx fR;
//    private DcMotorEx bL;
//    private DcMotorEx bR;
//    private List<DcMotorEx> allDriveMotors;
//    public void configureMotors() {
//        for (DcMotorEx motor : allDriveMotors) {
//            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        }
//    }

public void buildPaths() {
    /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
//    scanTag = new Path(new BezierLine(startPose, scanPose));
//        scanTag.setLinearHeadingInterpolation(startPose.getHeading(), scanPose.getHeading());

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
    CollectSet1_3 = follower.pathBuilder()
            .addPath(new BezierLine(Collect1_2, Collect1_3))
            .setLinearHeadingInterpolation(Collect1_2.getHeading(), Collect1_3.getHeading())
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();
    CollectSet1_4 = follower.pathBuilder()
            .addPath(new BezierLine(Collect1_3, Collect1_4))
            .setLinearHeadingInterpolation(Collect1_3.getHeading(), Collect1_4.getHeading())
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
                follower.followPath(scorePreload);
                launcherMotor.setPower(0.6);
                canopyServo.setPower(40);

                /* Score Preload */
                rliftServo.setPosition(0.5);
                try {
                    sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                rliftServo.setPosition(0.0);

                lliftServo.setPosition(0.01);
                try {
                    sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                lliftServo.setPosition(0.2) ;

                lliftServo.setPosition(0.01);
                try {
                    sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                lliftServo.setPosition(0.2) ;

                canopyServo.setPower(0);

                /* Score Preload End*/
                setPathState(10);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(grabPickup1, true);
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    setPathState(2);
                }
                break;
            case 2:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /*Start Intake Wheel*/

                    //intakeServo.setPower(-100);
                    intakeMotor.setPower(1);
                    canopyServo.setPower(40);
                    sorterServo.setPosition(0.1); // sorter left to grab purple balls
                    //launcherMotor.setPower(0.6);


                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(CollectSet1_1, true);
                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    sorterServo.setPosition(0.6); // sorter right to grab green ball
                    follower.followPath(CollectSet1_2, true);
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    //intakeServo.setPower(0);
                    intakeMotor.setPower(0);
                    canopyServo.setPower(0);

                    follower.followPath(CollectSet1_3, true);
                    follower.followPath(CollectSet1_4, true);
                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    setPathState(3);

                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1, true);
                    canopyServo.setPower(40);
                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }

                    /* Score First Set */
                    rliftServo.setPosition(.5);
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    rliftServo.setPosition(0);

                    lliftServo.setPosition(0.01);
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    lliftServo.setPosition(0.2) ;

                    lliftServo.setPosition(0.01);
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    lliftServo.setPosition(0.2) ;
                    canopyServo.setPower(0);
                    /* Score First Set End*/



                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2, true);
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    //intakeServo.setPower(100);
                    intakeMotor.setPower(1);
                    canopyServo.setPower(40);
                    sorterServo.setPosition(0.1);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(CollectSet2_1, true);
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    sorterServo.setPosition(0.6);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(CollectSet2_2, true);
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    sorterServo.setPosition(0.1);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(CollectSet2_3, true);
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    //intakeServo.setPower(0);
                    intakeMotor.setPower(0);
                    canopyServo.setPower(0);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scorePickup2, true);
                    canopyServo.setPower(40);
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }

                    /* Score Second Set */
                    rliftServo.setPosition(.5);
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    rliftServo.setPosition(0);

                    lliftServo.setPosition(0.01);
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    lliftServo.setPosition(0.2) ;

                    lliftServo.setPosition(0.01);
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    lliftServo.setPosition(0.2) ;

                    canopyServo.setPower(0);
                    /* Score Second Set End*/
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {


                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(grabPickup3, true);
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                        //intakeServo.setPower(100);
                        intakeMotor.setPower(1);
                        canopyServo.setPower(40);
                        sorterServo.setPosition(0.6);
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(CollectSet3_1, true);
                        try {
                            sleep(500);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                    sorterServo.setPosition(0.1);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(CollectSet3_2, true);
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }

                        //intakeServo.setPower(0);
                        intakeMotor.setPower(0);
                        canopyServo.setPower(0);
                    setPathState(9);
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scorePickup3, true);
                    canopyServo.setPower(40);
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }

                    /* Score Third Set */
                    rliftServo.setPosition(.5);
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    rliftServo.setPosition(0);

                    lliftServo.setPosition(0.01);
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    lliftServo.setPosition(0.2) ;

                    lliftServo.setPosition(0.01);
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    lliftServo.setPosition(0.2) ;

                    canopyServo.setPower(0);
                    launcherMotor.setPower(0);
                    /* Score Third Set End*/
                    setPathState(10);
                }
                break;

            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(Leave, true);
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    setPathState(11);
                }
                break;
            case 11:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    setPathState(-1);
                }

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

        //NormalizedRGBA colors = colorSensor.getNormalizedColors();
        //NormalizedColorSensor.NormalizedRGBA colors = colorSensor.getNormalizedColors();
//        float[] hsvValues = new float[3];
//        Color.colorToHSV(colors.toColor(), hsvValues);

        // Sorter Code

//        if (hsvValues[0] >= GREEN_HUE_MIN && hsvValues[0] <= GREEN_HUE_MAX)
//        {
//            sorterServo.setPosition(0.6);
//        }
//        else
//        {
//            sorterServo.setPosition(0.1);
//        }

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