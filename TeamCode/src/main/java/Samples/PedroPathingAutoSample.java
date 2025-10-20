package Samples;


import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "PedroPathingAuto", group = "Examples")
public class PedroPathingAutoSample extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose FirstPose = new Pose(12, 0, Math.toRadians(0));
    private final Pose SecondPose = new Pose(12, 12, Math.toRadians(0));
    private final Pose ThirdPose = new Pose(0, 12, Math.toRadians(0));

    private final Pose stopPose = new Pose(0, 12, Math.toRadians(0));

    private Path scorePreload;
    private PathChain First, Second, Third, Stop;

public void buildPaths() {

    scorePreload = new Path(new BezierLine(startPose, FirstPose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), FirstPose.getHeading());

    First = follower.pathBuilder()
            .addPath(new BezierLine(FirstPose, SecondPose))
                .setLinearHeadingInterpolation(FirstPose.getHeading(), SecondPose.getHeading())
            .build();

    Second = follower.pathBuilder()
            .addPath(new BezierLine(SecondPose, ThirdPose))
                .setLinearHeadingInterpolation(SecondPose.getHeading(), ThirdPose.getHeading())
            .build();

    Third = follower.pathBuilder()
            .addPath(new BezierLine(ThirdPose, startPose))
                .setLinearHeadingInterpolation(ThirdPose.getHeading(), startPose.getHeading())
            .build();

    Stop = follower.pathBuilder()
            .addPath(new BezierLine(ThirdPose, stopPose))
            .setLinearHeadingInterpolation(ThirdPose.getHeading(), stopPose.getHeading())
            .build();
}

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:

                if (!follower.isBusy()) {
                    follower.followPath(First, true);
                    try {
                        sleep(2000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    setPathState(2);

                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(Second, true);
                    try {
                        sleep(2000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    setPathState(3);
                }

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(Third, true);
                    try {
                        sleep(2000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(Stop, true);
                    try {
                        sleep(2000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
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

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}