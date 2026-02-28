package TeleOpCode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="MainTeleop_Pedro", group="Linear OpMode")
//@Disabled
public class MainTeleop_Pedro extends LinearOpMode {

    //Define the Limelight Object
    private Limelight3A limelight;
    private DcMotorEx launcherMotor;

    //============Pedro parameters===============
    private Follower follower;
    public static Pose startingPose;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    // Variables for speed control (Place these above your init() or loop())
    private boolean lastLeftBumper = false;
    private boolean slowMode = false;

    private double lockedHeading = 0; // Heading to maintain
    private boolean headingLockEnabled = false;
//================================
//PID Variables
    //*********************************************************************
    double integral = 0;
    double lastError = 0;
    double proportional = 0;
    double derivative = 0;
    double turnPower = 0;
    double targetAngleError = 0;
    private ElapsedTime runtime = new ElapsedTime();
    private Timer actionTimer;

    enum ServoState {READY, STEP_ONE, STEP_TWO}
    ServoState currentAction = ServoState.READY;
    ElapsedTime ServoTimer = new ElapsedTime();
    private boolean RClick = false;
    private boolean LClick = false;
    private double targetVelocity = 0;
    private IMU imu;
//    private double launcherspeed;
    private double launcherVelocity;
    // PID Constants
//    private final double POWER_INCREMENT = 0.05; // Increase by 5% each press
//    private final double MAX_POWER = 1.0; // 100% power
//    private final double MIN_POWER = -1.0; // Reverse full power
//

    //PIDF values (we're only tuning P and F, I and D stay at 0)
    private double p = 300.0;
    private double f = 13.65;

    //Velocity Constants
    private final double VELOCITY_INCREMENT = 25; // Increase by 25 each press
    private final double MAX_VELOCITY = 1675; // Maximum Velocity for far shots
    private final double MIN_VELOCITY = 1200; // Minimum Velocity for close shots

    private static final double Kp = 0.023; // Proportional Constant (needs Tuning)
    private static final double Ki = 0.0; // Integral Constant (optional, needs Tuning)
    private static final double Kd = 0.00001; //Derivative Constant (optional, need Tuning)
    private static final double TURN_TOLERANCE = 0.6; // Allowed error in degrees

    //*********************************************************************

//Distance Sensor variables
    //*********************************************************************
    private DigitalChannel LlaserInput;
    private DigitalChannel RlaserInput;
    //*********************************************************************

    // Limelight physical constants for distance calculation
    private static final double TARGET_HEIGHT_IN = 45.0; // Example target height
    private static final double CAMERA_HEIGHT_IN = 16.0; // Your camera height
    private static final double CAMERA_ANGLE_DEG = 0.0; // Your camera mounting angle


    private double calculateDistance(double ty) {
        // Angle to goal relative to the floor
        double angleToGoalRadians = Math.toRadians(CAMERA_ANGLE_DEG + ty);
        // Standard distance formula: d = (h2 - h1) / tan(angleToGoal)
        return (TARGET_HEIGHT_IN - CAMERA_HEIGHT_IN) / Math.tan(angleToGoalRadians);
    }

    /**
     * Maps the calculated distance to a required flywheel speed (RPM).
     * This mapping should be tuned through testing at various distances.
     * @param distance Distance to the target in inches.
     * @return Target RPM.
     */
    private double calculateTargetRPM(double distance) {
        // This is a simple linear example; you will likely need a more complex regression
        // (e.g., a lookup table or a quadratic equation) for accuracy across all distances
        // Example linear equation: y = mx + b
        // Adjust these values based on your robot's performance
        final double slope = 15.0;
        final double intercept = 1000.0;
        return (distance * slope) + intercept;
    }


    @Override
    public void runOpMode() throws InterruptedException {

//Using Pedropathing
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

        follower.startTeleopDrive();
//End


        DcMotor fL = hardwareMap.dcMotor.get("front_left_drive");
        DcMotor fR = hardwareMap.dcMotor.get("front_right_drive");
        DcMotor bL = hardwareMap.dcMotor.get("back_left_drive");
//        DcMotor launcher = hardwareMap.dcMotor.get("launcher");
//        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        DcMotor lifter = hardwareMap.dcMotor.get("Lifter");
        DcMotor lifter2 = hardwareMap.dcMotor.get("Lifter2");
        DcMotor bR = hardwareMap.dcMotor.get("back_right_drive");
        DcMotor grabber = hardwareMap.dcMotor.get("grabber");
        CRServo Canopy = hardwareMap.get(CRServo.class, "Canopy");
        Servo Rlift = hardwareMap.get(Servo.class, "Rlift");
        Servo Llift = hardwareMap.get(Servo.class, "Llift");
        Servo Sorter = hardwareMap.get(Servo.class, "Sorter");
        Servo HeadingLight = hardwareMap.get(Servo.class, "LED_light");
        limelight = hardwareMap.get(Limelight3A.class, "Limelight3A");
        limelight.setPollRateHz(100); // Set poll rate to 100Hz
        limelight.pipelineSwitch(3);  // AprilTags 20, 24
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

//LauncherMotor Initialization
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setDirection(DcMotorEx.Direction.REVERSE);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set initial PIDF coefficients (P, I, D, F)
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p,0,0,f);
        launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        LlaserInput = hardwareMap.get(DigitalChannel.class, "LDsensor");
        Servo LeftDLight = hardwareMap.get(Servo.class, "LDlight");
        // Set the channel as an input
        LlaserInput.setMode(DigitalChannel.Mode.INPUT);

        RlaserInput = hardwareMap.get(DigitalChannel.class, "RDsensor");
        Servo RightDLight = hardwareMap.get(Servo.class, "RDlight");
        // Set the channel as an input
        RlaserInput.setMode(DigitalChannel.Mode.INPUT);


        Servo Rgate = hardwareMap.get(Servo.class, "Rgate");

        // reverse the left side motors because some
        //meccanum wheels are backwards
        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.FORWARD);
        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        fR.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherMotor.setDirection(DcMotorEx.Direction.REVERSE);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lifter.setDirection(DcMotorSimple.Direction.REVERSE);


        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // The code after this is only run after start is pressed
        ElapsedTime timer = new ElapsedTime();


        if (isStopRequested()){
            return;
        }
        //This is loop that checks the gamepad for inputs every iteration
        boolean IntakeWheel = false ;
        boolean CrossCanopy = false ;
        boolean TriCanopy = false;

        waitForStart();

        // Variables to track button press state (to avoid rapid repeats)
        boolean lastAState = false;
        boolean lastBState = false;

        while (opModeIsActive())
        {

            //Using Pedropathing
            follower.update();
            telemetryM.update();
            launcherMotor.setVelocity(1520);


            // Get current robot pose
            Pose currentPose = follower.getPose();

            // Get robot velocity components (inches/sec)
            double velocityX = follower.poseTracker.getLocalizer().getVelocity().getX();
            double velocityY = follower.poseTracker.getLocalizer().getVelocity().getY();

            // 1. Detect the "Rising Edge" (click) of the left bumper
            if (gamepad1.left_bumper && !lastLeftBumper) {
                slowMode = !slowMode; // Flip the slowMode state
            }
            lastLeftBumper = gamepad1.left_bumper; // Update last state

            // 2. Determine speed multiplier based on the toggle
            double driveSpeed = slowMode ? 0.2 : 1.0;
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * driveSpeed,
                        -gamepad1.left_stick_x * driveSpeed,
                        -gamepad1.right_stick_x * driveSpeed,
                        true // Robot Centric
                );
              follower.update();

            telemetryM.debug("position", follower.getPose());
            telemetryM.debug("velocity", follower.getVelocity());

            int lastPos = launcherMotor.getCurrentPosition();
            timer.reset();
            runtime.reset();
            double dt = timer.seconds();

            int curPos = launcherMotor.getCurrentPosition();
            int dPos = curPos - lastPos;

            double ticksPerSec = dPos / dt;
            double ticksPerRev = 28.0;  // change this to the correct value
            double rpm = Math.abs((ticksPerSec / ticksPerRev) * 60.0);

// Correct RPM formula

            telemetry.addData("RPM", rpm);
            telemetry.update();

//            lastPos = curPos;
            timer.reset();

            limelight.start();

            // Read the sensor state (true = HIGH, false = LOW)
            boolean LstateHigh = LlaserInput.getState();

            // Active-HIGH: HIGH means an object is detected
            boolean Ldetected = LstateHigh;
            // Display detection state
            if (Ldetected) {
                telemetry.addLine("Object detected!");
                LeftDLight.setPosition(0.600);
            } else {
                telemetry.addLine("No object detected");
                LeftDLight.setPosition(0);
            }

            // Display the raw HIGH/LOW signal for reference
            telemetry.addData("Raw (HIGH/LOW)", LstateHigh);
            telemetry.update();

            // Read the sensor state (true = HIGH, false = LOW)
            boolean RstateHigh = RlaserInput.getState();

            // Active-HIGH: HIGH means an object is detected
            boolean Rdetected = RstateHigh;
            // Display detection state
            if (Rdetected) {
                telemetry.addLine("Object detected!");
                RightDLight.setPosition(0.600);
            } else {
                telemetry.addLine("No object detected");
                RightDLight.setPosition(0);
            }
            // Display the raw HIGH/LOW signal for reference
            telemetry.addData("Raw (HIGH/LOW)", RstateHigh);
            telemetry.update();

            double turnSpeed = 0.0;

            // Ensure the Limelight has a valid target (tv = 1)

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());
            LLResult llResult = limelight.getLatestResult();

            double ty = 0;

            if (llResult != null && llResult.isValid()) {

                Pose3D botPose = llResult.getBotpose_MT2();
                // Get the horizontal offset from the crosshair in degrees (-27 to 27)
                double tx = llResult.getTx(); // FTC
                ty = llResult.getTy(); // vertical distance

                // A higher tx means the target is to the right, so we need to turn right (positive speed)
                if (tx > -2.0 && tx < 2.0)
                {
                    HeadingLight.setPosition(0.500);
                }
                else if (tx > -23.0 && tx < 23.0 ) {
                    HeadingLight.setPosition(0.388);
                }
                else
                {
                    HeadingLight.setPosition(0.277);
                }

            } else {
                // No target found, stop turning
                HeadingLight.setPosition(0.0);
            }

            double distance = calculateDistance(ty);
            double targetRPM = calculateTargetRPM(distance);

            // Set flywheel velocity
//            launcherMotor.setVelocity(targetRPM/2, AngleUnit.DEGREES); // or use other AngleUnit

            // Telemetry for tuning/monitoring
            telemetry.addData("Ty", ty);
            telemetry.addData("Distance (Inches)", distance);
            telemetry.addData("position:", follower.getPose());
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Current Velocity (deg/s)", launcherMotor.getVelocity(AngleUnit.DEGREES));
            telemetry.update();


    /*
GamePad 1
    Triangle - Intake - forward
    Cross - intake - reverse
    Square - Sorter Left
    Circle - Sorter Right
    right_bumper - Auto Heading
    dpad-up - Sliders up
    dpad-down - Sliders down
 */


            //Apply PIDF coefficients every loop
            //PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p,0,0,f);
            launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


            if (gamepad1.right_bumper)  //Auto heading to Target.
            {
                 targetAngleError = llResult.getTx(); // Horizontal offset from Limelight

                // PID variables

                // Simple PID control for rotation
                proportional = Kp * targetAngleError;
                integral += Ki * targetAngleError;
                derivative = Kd * (targetAngleError - lastError);
                lastError = targetAngleError;

                // Calculate the turning speed using PID values
                 turnPower = proportional + integral + derivative;

                // If error is small enough, stop turning
                if (Math.abs(targetAngleError) < 1.0) {
                    turnPower = 0;
                }

                telemetry.addData("turn power", turnPower);
                telemetry.update();
                // Apply differential power to rotate robot

//                double maxTurnPower = Math.min(Math.abs(turnPower * 10), 1.0); // Ensure max power = 1.0

                 bL.setPower(turnPower*10);
                 fL.setPower(turnPower*10);
                 bR.setPower(-turnPower*10);
                 fR.setPower(-turnPower*10);

//                bL.setPower(maxTurnPower);
//                fL.setPower(maxTurnPower);
//                bR.setPower(-maxTurnPower);
//                fR.setPower(-maxTurnPower);

            }

//Code for Gates

            if (gamepad2.right_stick_button)   // Open the Gate
            {
//               grabber.setPower(-0.3);
                Rgate.setPosition(0.18);
            }

            if (gamepad2.left_stick_button)   // Close the Gate
            {
                Rgate.setPosition(0);
//                grabber.setPower(-1);
            }


// Code for Intake

                if (gamepad1.triangle)   // Intake Wheel Inward
                {
                        grabber.setPower(-1);
                }

                if (gamepad1.cross)   // Intake wheel reverse
                {
                        grabber.setPower(1);
                }

// canopy code
            if (gamepad2.triangle)   // canopy Wheel turn on
            {
                Canopy.setPower(-1);   // Turn on canopy
            }

            if (gamepad2.cross)   // canopy wheel turn off
            {
                Canopy.setPower(0);  // Turn off canopy
            }
//// Launcher speed increase
//            if (gamepad2.circle && !lastAState)   // Launcher speed increment
//            {
//                launcherspeed = launcherMotor.getPower() + POWER_INCREMENT;
//                if (launcherspeed > MAX_POWER)  launcherspeed = MAX_POWER;
//                // Apply power to motor
//                launcherMotor.setPower(launcherspeed);
//            }
//            if (gamepad2.square && !lastBState)   // Launcher speed decrease
//            {
//                launcherspeed = launcherMotor.getPower() - POWER_INCREMENT;
//                if (launcherspeed < MIN_POWER) launcherspeed = MIN_POWER;
//                launcherMotor.setPower(launcherspeed);
//            }
//
//            // Save button states for next loop
//            lastAState = gamepad2.circle;
//            lastBState = gamepad2.square;

// Launcher velocity control
            if (gamepad2.circle && !lastAState)   // Launcher Velocity increment
            {
                launcherVelocity = launcherMotor.getVelocity() + VELOCITY_INCREMENT;
                if (launcherVelocity > MAX_VELOCITY)  launcherVelocity = MAX_VELOCITY;
                // Apply Velocity to motor
                launcherMotor.setVelocity(launcherVelocity);
            }
            if (gamepad2.square && !lastBState)   // Launcher Velocity decrease
            {
                launcherVelocity = launcherMotor.getPower() - VELOCITY_INCREMENT;
                if (launcherVelocity < MIN_VELOCITY) launcherVelocity = MIN_VELOCITY;
                launcherMotor.setVelocity(launcherVelocity);
            }

            // Save button states for next loop
            lastAState = gamepad2.circle;
            lastBState = gamepad2.square;


// Sorter Code

            if (gamepad1.square)   // Sorter Left to Grab Purple Balls
            {
                Sorter.setPosition(0.10);
            }
            if (gamepad1.circle)   // Sorter Right to Grab Green Balls
            {
                Sorter.setPosition(0.7);
            }

            if (gamepad2.dpad_up)
            {
//            launcherMotor.setPower(0.75);
//                targetVelocity = 1875; // speed (ticks/sec)
                targetVelocity = 1550; // speed (ticks/sec)
            }

            if (gamepad2.dpad_down)
            {
//            launcherMotor.setPower(0.55);
//                targetVelocity = 1280; // High speed (ticks/sec)
                targetVelocity = 1380; // High speed (ticks/sec)
            }

            if (gamepad2.dpad_left)
            {
//            launcherMotor.setPower(0.6);
//                targetVelocity = 1500; // High speed (ticks/sec)
//                targetVelocity = 1380; // High speed (ticks/sec)
                targetVelocity = 1500; // High speed (ticks/sec)
            }

            if (gamepad2.dpad_right)
            {
//            launcherMotor.setPower(0.85);
//                targetVelocity = 2500; // High speed (ticks/sec) ASSUMING MAX VELOCITY
//                targetVelocity = 2125; // High speed (ticks/sec)
                targetVelocity = 1675; // High speed (ticks/sec)
            }

            launcherMotor.setVelocity(targetVelocity);

            // 3. Update Follower and Telemetry
            follower.update();


 switch(currentAction){
     case READY:
         if(gamepad2.right_bumper && !gamepad2.left_bumper)
         {
             RClick = true;
             LClick = false;
             Rlift.setPosition(0.24);
             ServoTimer.reset();
             currentAction = ServoState.STEP_ONE;
         }
         if(!gamepad2.right_bumper && gamepad2.left_bumper)
         {
             RClick = false;
             LClick = true;
             Llift.setPosition(0.01);
             ServoTimer.reset();
             currentAction = ServoState.STEP_ONE;
         }
          break;
     case STEP_ONE:
         if(ServoTimer.milliseconds() >= 200 && !LClick && RClick)
         {
             Rlift.setPosition(0.05);
             ServoTimer.reset();
             currentAction = ServoState.STEP_TWO;
         }
         if(ServoTimer.milliseconds() >= 200 && LClick && !RClick)
         {
             Llift.setPosition(0.2);
             ServoTimer.reset();
             currentAction = ServoState.STEP_TWO;
         }
         break;

     case STEP_TWO:
         if(!gamepad2.right_bumper && !gamepad2.left_bumper)
         {
             RClick = false;
             LClick = false;
             currentAction = ServoState.READY;
         }
         break;
 }


//            if (gamepad2.right_bumper)
//            {
//                actionTimer.resetTimer();
//
//                Rlift.setPosition(0.24);
//                if(actionTimer.getElapsedTimeSeconds() > 0.2){
//                    Rlift.setPosition(0.05);
//                }
//            }
//
//            if (gamepad2.left_bumper)
//            {
//                actionTimer.resetTimer();
//
//                Llift.setPosition(0.01);
//                if(actionTimer.getElapsedTimeSeconds() > 0.2){
//                    Llift.setPosition(0.2);
//                }
//            }


//            if (gamepad2.rightBumperWasPressed())
//            {
//                Rlift.setPosition(0.24);
//            }
//
//            if (gamepad2.rightBumperWasReleased())
//            {
//                Rlift.setPosition(0.05);
//            }
//
//            if (gamepad2.leftBumperWasPressed())
//            {
//                Llift.setPosition(0.01);
//            }
//
//            if (gamepad2.leftBumperWasReleased())
//            {
//                Llift.setPosition(0.2);
//            }

//            if (gamepad2.right_bumper)
//            {
//                Rlift.setPosition(0.24);
//                sleep(200);
//                Rlift.setPosition(0.05);
//            }
//            if (gamepad2.left_bumper)
//            {
//                Llift.setPosition(0.01);
//                sleep(200);
//                Llift.setPosition(0.2) ;
//            }

            if (gamepad1.dpad_up)
            {
                lifter.setPower(-1);
                lifter2.setPower(-1);
            }
            else
            {
                lifter.setPower(0);
                lifter2.setPower(0);
            }
            if (gamepad1.dpad_down)
            {
                lifter.setPower(1);
                lifter2.setPower(1);
            }
            else
            {
                lifter.setPower(0);
                lifter2.setPower(0);
            }

//            // Drive Wheel calculations
//
//            double y = -gamepad1.left_stick_y;
//            double x = gamepad1.left_stick_x*1.1;
//            double rx = gamepad1.right_stick_x;
//
//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//
//            double frontLeftPower  = ((y + x + rx)/denominator);
//            double frontRightPower = ((y - x - rx)/denominator);
//            double backLeftPower   =  ((y - x + rx)/denominator);
//            double backRightPower  = ((y + x - rx)/denominator);
//
//            fL.setPower(frontLeftPower);
//            fR.setPower(frontRightPower);
//            bL.setPower(backLeftPower);
//            bR.setPower(backRightPower);
        }

    }
}