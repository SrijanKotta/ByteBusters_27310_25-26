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
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;


@Autonomous(name = "Decode_Auto1_Red_GPP", group = "Examples")
public class Decode_Auto1_Red_GPP extends OpMode {
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
    //private NormalizedColorSensor colorSensor;

//    private final float GREEN_HUE_MIN = 80;
//    private final float GREEN_HUE_MAX = 145;



    private final Pose startPose = new Pose(115.5, 128, Math.toRadians(55)); // Start Pose of our robot.
    private final Pose scanPose = new Pose(72, 128, Math.toRadians(90)); // Scan Pose of our robot for april Tag. It is facing the AprilTag at a 90 degree angle.

    private final Pose scorePose = new Pose(84, 84, Math.toRadians(50)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePose1 = new Pose(84, 84, Math.toRadians(50)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private final Pose pickup1Pose = new Pose(106, 84, Math.toRadians(10)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose Collect1_1 = new Pose(119, 84, Math.toRadians(10));
    private final Pose Collect1_2 = new Pose(126, 84, Math.toRadians(10));
//    private final Pose Collect1_3 = new Pose(114, 74, Math.toRadians(90));
//    private final Pose Collect1_4 = new Pose(120, 74, Math.toRadians(90));
    private final Pose pickup2Pose = new Pose(106, 60, Math.toRadians(10)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose Collect2_1 = new Pose(114, 60, Math.toRadians(10));
    private final Pose Collect2_2 = new Pose(119, 60, Math.toRadians(10));
    private final Pose Collect2_3 = new Pose(126, 60, Math.toRadians(10));
    private final Pose pickup3Pose = new Pose(106, 36, Math.toRadians(10)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose Collect3_1 = new Pose(114, 36, Math.toRadians(10));
    private final Pose Collect3_2 = new Pose(126, 36, Math.toRadians(10));
    private final Pose LeavePose = new Pose(106, 84, Math.toRadians(10)); // Leave Pose at the end of Auto.

    //    private Path scanTag;
    private Path scorePreload;
//    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, Leave;
    //private PathChain scorePreload1, grabPickup1, grabPickup1_1, CollectSet1_1, CollectSet1_1_1, CollectSet1_2, CollectSet1_2_1, CollectSet1_3, CollectSet1_3_1, CollectSet1_4, CollectSet1_4_1, scorePickup1, scorePickup1_1, scorePickup1_1_1, grabPickup2, grabPickup2_1, CollectSet2_1, CollectSet2_1_1, CollectSet2_2, CollectSet2_2_1, CollectSet2_3, CollectSet2_3_1, scorePickup2, scorePickup2_1, grabPickup3, CollectSet3_1, CollectSet3_2, scorePickup3, Leave;

//    private PathChain scorePreload1, grabPickup1, grabPickup1_1, CollectSet1_1, CollectSet1_1_1, CollectSet1_2, CollectSet1_2_1, CollectSet1_3, CollectSet1_3_1, CollectSet1_4, CollectSet1_4_1, scorePickup1, scorePickup1_1, grabPickup2, grabPickup2_1, CollectSet2_1, CollectSet2_1_1, CollectSet2_2, CollectSet2_2_1, CollectSet2_3, CollectSet2_3_1, scorePickup2, scorePickup2_1, grabPickup3, CollectSet3_1, CollectSet3_2, scorePickup3, Leave,Leave1;
private PathChain scorePreload1, grabPickup1, grabPickup1_1, CollectSet1_1, CollectSet1_1_1, CollectSet1_2, CollectSet1_2_1, scorePickup1, scorePickup1_1, grabPickup2, grabPickup2_1, CollectSet2_1, CollectSet2_1_1, CollectSet2_2, CollectSet2_2_1, CollectSet2_3, CollectSet2_3_1, scorePickup2, scorePickup2_1, grabPickup3, CollectSet3_1, CollectSet3_2, scorePickup3, Leave,Leave1;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
//    scanTag = new Path(new BezierLine(startPose, scanPose));
//        scanTag.setLinearHeadingInterpolation(startPose.getHeading(), scanPose.getHeading());

        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePreload1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, scorePose1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), scorePose1.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), pickup1Pose.getHeading())
                .build();

        grabPickup1_1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, pickup1Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1Pose.getHeading())
                .build();

        CollectSet1_1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, Collect1_1))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), Collect1_1.getHeading())
                .build();
        CollectSet1_1_1 = follower.pathBuilder()
                .addPath(new BezierLine(Collect1_1, Collect1_1))
                .setLinearHeadingInterpolation(Collect1_1.getHeading(), Collect1_1.getHeading())
                .build();
        CollectSet1_2 = follower.pathBuilder()
                .addPath(new BezierLine(Collect1_1, Collect1_2))
                .setLinearHeadingInterpolation(Collect1_1.getHeading(), Collect1_2.getHeading())
                .build();
        CollectSet1_2_1 = follower.pathBuilder()
                .addPath(new BezierLine(Collect1_2, Collect1_2))
                .setLinearHeadingInterpolation(Collect1_2.getHeading(), Collect1_2.getHeading())
                .build();
//        CollectSet1_3 = follower.pathBuilder()
//                .addPath(new BezierLine(Collect1_2, Collect1_3))
//                .setLinearHeadingInterpolation(Collect1_2.getHeading(), Collect1_3.getHeading())
//                .build();
//        CollectSet1_3_1 = follower.pathBuilder()
//                .addPath(new BezierLine(Collect1_3, Collect1_3))
//                .setLinearHeadingInterpolation(Collect1_3.getHeading(), Collect1_3.getHeading())
//                .build();
//        CollectSet1_4 = follower.pathBuilder()
//                .addPath(new BezierLine(Collect1_3, Collect1_4))
//                .setLinearHeadingInterpolation(Collect1_3.getHeading(), Collect1_4.getHeading())
//                .build();
//
//        CollectSet1_4_1 = follower.pathBuilder()
//                .addPath(new BezierLine(Collect1_4, Collect1_4))
//                .setLinearHeadingInterpolation(Collect1_4.getHeading(), Collect1_4.getHeading())
//                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(Collect1_2, scorePose1))
                .setLinearHeadingInterpolation(Collect1_2.getHeading(), scorePose.getHeading())
                .build();
        scorePickup1_1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, scorePose1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), scorePose1.getHeading())
                .build();


        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        grabPickup2_1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, pickup2Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2Pose.getHeading())
                .build();


        CollectSet2_1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, Collect2_1))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), Collect2_1.getHeading())
                .build();
        CollectSet2_1_1 = follower.pathBuilder()
                .addPath(new BezierLine(Collect2_1, Collect2_1))
                .setLinearHeadingInterpolation(Collect2_1.getHeading(), Collect2_1.getHeading())
                .build();
        CollectSet2_2 = follower.pathBuilder()
                .addPath(new BezierLine(Collect2_1, Collect2_2))
                .setLinearHeadingInterpolation(Collect2_1.getHeading(), Collect2_2.getHeading())
                .build();
        CollectSet2_2_1 = follower.pathBuilder()
                .addPath(new BezierLine(Collect2_2, Collect2_2))
                .setLinearHeadingInterpolation(Collect2_2.getHeading(), Collect2_2.getHeading())
                .build();
        CollectSet2_3 = follower.pathBuilder()
                .addPath(new BezierLine(Collect2_2, Collect2_3))
                .setLinearHeadingInterpolation(Collect2_2.getHeading(), Collect2_3.getHeading())
                .build();

        CollectSet2_3_1 = follower.pathBuilder()
                .addPath(new BezierLine(Collect2_3, Collect2_3))
                .setLinearHeadingInterpolation(Collect2_3.getHeading(), Collect2_3.getHeading())
                .build();
        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(Collect2_3, scorePose1))
                .setLinearHeadingInterpolation(Collect2_3.getHeading(), scorePose1.getHeading())
                .build();

        scorePickup2_1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, scorePose1))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), scorePose1.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();
        CollectSet3_1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, Collect3_1))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), Collect3_1.getHeading())
                .build();
        CollectSet3_2 = follower.pathBuilder()
                .addPath(new BezierLine(Collect3_1, Collect3_2))
                .setLinearHeadingInterpolation(Collect3_1.getHeading(), Collect3_2.getHeading())
                .build();
        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(Collect3_2, scorePose))
                .setLinearHeadingInterpolation(Collect3_2.getHeading(), scorePose.getHeading())
                .build();

        Leave = follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, LeavePose))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), LeavePose.getHeading())
                .build();
        Leave1 = follower.pathBuilder()
                .addPath(new BezierLine(LeavePose, LeavePose))
                .setLinearHeadingInterpolation(LeavePose.getHeading(), LeavePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                launcherMotor.setPower(-0.7);
                intakeMotor.setPower(1);
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePreload1, true);
                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    setPathState(2);
                }
                break;

            case 2:
                canopyServo.setPower(-40);

                try {
                    sleep(1500);
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
                launcherMotor.setPower(-0.75);

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
                lliftServo.setPosition(0.2) ;

                launcherMotor.setPower(-0.75);

                try {
                    sleep(2000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                lliftServo.setPosition(0.01);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                lliftServo.setPosition(0.2) ;

                /* Score Preload End*/
                setPathState(3);

                break;

            case 3:
                follower.followPath(grabPickup1, true);
                /* Score Preload End*/

                setPathState(4);

                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(grabPickup1_1, true);

                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    setPathState(5);
                }
                break;
            case 5:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if (!follower.isBusy()) {
                /*Start Intake Wheel*/
                Constants.driveConstants.maxPower(0.5);
                sorterServo.setPosition(0.15); // sorter left to grab purple balls
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                setPathState(6);
                break;
            case 6:
                follower.followPath(CollectSet1_1, true);

                setPathState(7);
                break;
            case 7:
                if (!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */

                    follower.followPath(CollectSet1_1_1, true);

                }
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                sorterServo.setPosition(0.62); // sorter right to grab green ball
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                setPathState(8);
                break;
            case 8:
                follower.followPath(CollectSet1_2, true);

                setPathState(9);
                break;
            case 9:
                if (!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(CollectSet1_2_1, true);


                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    Constants.driveConstants.maxPower(1);
                    setPathState(17);
                }
                break;
            case 10:

//                follower.followPath(CollectSet1_3, true);
                setPathState(11);
                break;
            case 11:
                if (!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(CollectSet1_3_1, true);
                    setPathState(12);
                }

                break;
            case 12:
//                follower.followPath(CollectSet1_4, true);
                setPathState(13);
                break;
            case 13:
                if (!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(CollectSet1_4_1, true);


                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    setPathState(14);

                }
                break;
            case 14:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
//                if (!follower.isBusy()) {

                /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                follower.followPath(scorePickup1, true);

                setPathState(15);
                break;
            case 15:

                if (!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1_1, true);
                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    setPathState(16);
                }
                break;
            case 16:
                launcherMotor.setPower(-0.7);
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
                launcherMotor.setPower(-0.75);

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
                lliftServo.setPosition(0.2) ;

                launcherMotor.setPower(-0.75);

                try {
                    sleep(2000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                lliftServo.setPosition(0.01);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                lliftServo.setPosition(0.2) ;

                  /* Score First Set End*/

                setPathState(99);
//                }
                break;
            case 17:
                follower.followPath(grabPickup2, true);
                setPathState(18);
                break;
            case 18:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2_1, true);
                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    setPathState(19);
                }
                break;
            case 19:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */

                canopyServo.setPower(-40);
                sorterServo.setPosition(0.15);
                /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                setPathState(20);
                break;
            case 20:
                Constants.driveConstants.maxPower(0.5);
                follower.followPath(CollectSet2_1, true);
                setPathState(21);
                break;
            case 21:
                if (!follower.isBusy()) {
                    follower.followPath(CollectSet2_1_1, true);

                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    sorterServo.setPosition(0.62);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    setPathState(22);
                }
                break;
            case 22:

                follower.followPath(CollectSet2_2, true);
                setPathState(23);
                break;
            case 23:

                if (!follower.isBusy()) {
                    follower.followPath(CollectSet2_2_1, true);


                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    sorterServo.setPosition(0.15);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    setPathState(24);
                }
                break;
            case 24:
                follower.followPath(CollectSet2_3, true);
                setPathState(25);
                break;
            case 25:
                if (!follower.isBusy()) {
                    follower.followPath(CollectSet2_3_1, true);

                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    Constants.driveConstants.maxPower(1);
                    setPathState(26);
                }
                break;
            case 26:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if (!follower.isBusy()) {
                /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                follower.followPath(scorePickup2, true);
                setPathState(27);
                break;
            case 27:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2_1, true);
                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    setPathState(28);
                }
                break;
            case 28:
                launcherMotor.setPower(-0.7);
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
                launcherMotor.setPower(-0.75);

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
                lliftServo.setPosition(0.2) ;

                launcherMotor.setPower(-0.75);

                try {
                    sleep(2000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                lliftServo.setPosition(0.01);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                lliftServo.setPosition(0.2) ;

                /* Score Second Set End*/
                setPathState(99);
//                }
                break;
            case 29:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
//                if (!follower.isBusy()) {


                /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                follower.followPath(grabPickup3, true);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                setPathState(99);
//                }
                break;
            case 30:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
//                if (!follower.isBusy()) {
                //intakeServo.setPower(100);
                intakeMotor.setPower(1);
                canopyServo.setPower(-40);
                sorterServo.setPosition(0.62);
                /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                follower.followPath(CollectSet3_1, true);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                sorterServo.setPosition(0.15);
                /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                follower.followPath(CollectSet3_2, true);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                //intakeServo.setPower(0);
                intakeMotor.setPower(0);
                setPathState(31);
//                }
                break;
            case 31:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if (!follower.isBusy()) {
                /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                follower.followPath(scorePickup3, true);
                launcherMotor.setPower(-0.7);
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
                launcherMotor.setPower(-0.75);

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
                lliftServo.setPosition(0.2) ;

                launcherMotor.setPower(-0.75);

                try {
                    sleep(2000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                lliftServo.setPosition(0.01);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                lliftServo.setPosition(0.2) ;

                canopyServo.setPower(0);
                launcherMotor.setPower(0);
                /* Score Third Set End*/
                setPathState(32);
//                }
                break;

            case 99:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                //if (!follower.isBusy()) {

                /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                follower.followPath(Leave, true);
                setPathState(100);
                //}
                break;
            case 100:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(Leave1, true);
                    setPathState(101);
                }
                break;
            case 101:
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