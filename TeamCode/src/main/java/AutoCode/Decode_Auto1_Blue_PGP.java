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


//Color sensor libraries
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import android.graphics.Color;

@Autonomous(name = "Decode_Auto1_Blue_PGP", group = "Examples")
public class Decode_Auto1_Blue_PGP extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final float GREEN_HUE_MIN = 80;
    private final float GREEN_HUE_MAX = 145;

    private final Pose startPose = new Pose(28.5, 128, Math.toRadians(135)); // Start Pose of our robot.
    private final Pose scanPose = new Pose(72, 128, Math.toRadians(90)); // Scan Pose of our robot for april Tag. It is facing the AprilTag at a 90 degree angle.
    private final Pose scorePose = new Pose(60, 84, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(44, 84, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose Collect1 = new Pose(14, 84, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(44, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose Collect2 = new Pose(14, 60, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(44, 36, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose Collect3 = new Pose(14, 36, Math.toRadians(180));
    private final Pose LeavePose = new Pose(44, 84, Math.toRadians(90)); // Leave Pose at the end of Auto.

    private Path scanTag;
    private Path scorePreload;
    //private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, Leave;
    private PathChain grabPickup1, CollectSet1, scorePickup1, grabPickup2, CollectSet2, scorePickup2, grabPickup3, CollectSet3, scorePickup3, Leave;
    CRServo Intake = hardwareMap.get(CRServo.class, "AxonServo");
    CRServo Canopy = hardwareMap.get(CRServo.class, "Canopy");
    Servo Rlift = hardwareMap.get(Servo.class, "Rlift");
    Servo Llift = hardwareMap.get(Servo.class, "Llift");
    Servo Sorter = hardwareMap.get(Servo.class, "Sorter");
    DcMotor launcher = hardwareMap.dcMotor.get("launcher");
    NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

    public void buildPaths() {
    /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
    scanTag = new Path(new BezierLine(startPose, scanPose));
        scanTag.setLinearHeadingInterpolation(startPose.getHeading(), scanPose.getHeading());
//    scorePreload.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading());
     */

//    scorePreload = new Path(new BezierLine(startPose, scorePose));
//    scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
//    scorePreload.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

    /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
/*
    scorePreload = follower.pathBuilder()
            .addPath(new BezierLine(scanPose, scorePose))
            .setLinearHeadingInterpolation(scanPose.getHeading(), scorePose.getHeading())
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();
*/

    scorePreload = new Path(new BezierLine(startPose, scorePose));
    scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
/*
    scorePreload = follower.pathBuilder()
            .addPath(new BezierLine(startPose, scorePose))
            .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();
*/
    /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
    grabPickup1 = follower.pathBuilder()
            .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    CollectSet1 = follower.pathBuilder()
            .addPath(new BezierLine(pickup1Pose, Collect1))
            .setLinearHeadingInterpolation(pickup1Pose.getHeading(), Collect1.getHeading())
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
    scorePickup1 = follower.pathBuilder()
            .addPath(new BezierLine(Collect1, scorePose))
                .setLinearHeadingInterpolation(Collect1.getHeading(), scorePose.getHeading())
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

   /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
    grabPickup2 = follower.pathBuilder()
            .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    CollectSet2 = follower.pathBuilder()
            .addPath(new BezierLine(pickup2Pose, Collect2))
            .setLinearHeadingInterpolation(pickup2Pose.getHeading(), Collect2.getHeading())
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();
    /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
    scorePickup2 = follower.pathBuilder()
            .addPath(new BezierLine(Collect2, scorePose))
                .setLinearHeadingInterpolation(Collect2.getHeading(), scorePose.getHeading())
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
    grabPickup3 = follower.pathBuilder()
            .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();
    CollectSet3 = follower.pathBuilder()
            .addPath(new BezierLine(pickup3Pose, Collect3))
            .setLinearHeadingInterpolation(pickup3Pose.getHeading(), Collect3.getHeading())
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();
    /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
    scorePickup3 = follower.pathBuilder()
            .addPath(new BezierLine(Collect3, scorePose))
                .setLinearHeadingInterpolation(Collect3.getHeading(), scorePose.getHeading())
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
                launcher.setPower(0.6);
                Canopy.setPower(40);


                /* Score Preload */
                Llift.setPosition(0.01);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                Llift.setPosition(0.2) ;

                Rlift.setPosition(1.5);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                Rlift.setPosition(0.92);

                 Llift.setPosition(0.01);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                Llift.setPosition(0.2) ;

                Canopy.setPower(0);

                /* Score Preload End*/
                setPathState(1);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(grabPickup1, true);
                    try {
                        sleep(2000);
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
                    Intake.setPower(100);
                    Canopy.setPower(40);
                    //launcher.setPower(0.6);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(CollectSet1, true);
                    try {
                        sleep(2000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }

                    Intake.setPower(0);
                    Canopy.setPower(0);
                    setPathState(3);

                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1, true);
                    Canopy.setPower(40);
                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }

                    /* Score First Set */
                    Llift.setPosition(0.01);
                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    Llift.setPosition(0.2) ;

                    Rlift.setPosition(1.5);
                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    Rlift.setPosition(0.92);

                    Llift.setPosition(0.01);
                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    Llift.setPosition(0.2) ;
                    Canopy.setPower(0);
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
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    Intake.setPower(100);
                    Canopy.setPower(40);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(CollectSet2, true);
                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    Intake.setPower(0);
                    Canopy.setPower(0);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scorePickup2, true);
                    Canopy.setPower(40);
                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }

                    /* Score Second Set */
                    Llift.setPosition(0.01);
                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    Llift.setPosition(0.2) ;

                    Rlift.setPosition(1.5);
                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    Rlift.setPosition(0.92);

                    Llift.setPosition(0.01);
                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    Llift.setPosition(0.2) ;

                    Canopy.setPower(0);
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
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                        Intake.setPower(100);
                        Canopy.setPower(40);
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(CollectSet3, true);
                        try {
                            sleep(1000);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                        Intake.setPower(0);
                        Canopy.setPower(0);
                    setPathState(9);
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scorePickup3, true);
                    Canopy.setPower(40);
                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }

                    /* Score Third Set */
                    Llift.setPosition(0.01);
                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    Llift.setPosition(0.2) ;

                    Rlift.setPosition(1.5);
                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    Rlift.setPosition(0.92);

                    Llift.setPosition(0.01);
                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    Llift.setPosition(0.2) ;

                    Canopy.setPower(0);
                    launcher.setPower(0);
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
                        sleep(1000);
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
                        sleep(2000);
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

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        //NormalizedColorSensor.NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);

        // Sorter Code

        if (hsvValues[0] >= GREEN_HUE_MIN && hsvValues[0] <= GREEN_HUE_MAX)
        {
            Sorter.setPosition(0.1);
        }
        else
        {
            Sorter.setPosition(0.6);
        }

    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


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