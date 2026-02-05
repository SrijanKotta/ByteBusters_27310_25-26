package TeleOpCode;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="MainTeleop", group="Linear OpMode")
//@Disabled
public class MainTeleop extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();

    private IMU imu;
    private double launcherspeed;
    private final double POWER_INCREMENT = 0.05; // Increase by 5% each press
    private final double MAX_POWER = 1.0; // 100% power
    private final double MIN_POWER = -1.0; // Reverse full power


    //Define the Limelight Object
    private Limelight3A limelight;
    // PID Constants
    private static final double Kp = 0.023; // Proportional Constant (needs Tuning)
    private static final double Ki = 0.0; // Integral Constant (optional, needs Tuning)
    private static final double Kd = 0.00001; //Derivative Constant (optional, need Tuning)
    private static final double TURN_TOLERANCE = 0.6; // Allowed error in degrees

    private Follower driveTrain;

    // Your measured max velocity at full power
    private static final double MAX_VELOCITY = 2000.0;

    // Shooter speed (adjustable from 0.0 to 1.0)
    private double shooterSpeed = 0.5;

    // Target velocities
    private double targetVelocity1 = 0.0;

    // PID variables
    private double integralSum1 = 0.0;
    private double lastError1 = 0.0;


    // Shooter management
    private boolean shootersOn = true;  // CHANGED: start with shooters ON
    private boolean prevRT1 = false;
    private boolean prevLT1 = false;

    // Gamepad 2 controls
    private boolean prevx2 = false;
    private boolean prevB2 = false;
    private static final double TRIGGER_THRESH = 0.2;
    private ElapsedTime pidTimer = new ElapsedTime();

    private DigitalChannel LlaserInput;
    private DigitalChannel RlaserInput;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor fL = hardwareMap.dcMotor.get("front_left_drive");
        DcMotor fR = hardwareMap.dcMotor.get("front_right_drive");
        DcMotor bL = hardwareMap.dcMotor.get("back_left_drive");
        DcMotor launcher = hardwareMap.dcMotor.get("launcher");
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
        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

            final double TICKS_PER_REV = 28.0;   // your encoder resolution
            final double MIN_DT = 0.03;         // minimum sample window (seconds) — 30 ms
            final double SMOOTH_ALPHA = 0.2;    // smoothing factor (0 = no smoothing, 1 = no smoothing retention)

            double max;
            double speed = 0.6;
            int lastPos = launcher.getCurrentPosition();
            timer.reset();
            runtime.reset();
            double dt = timer.seconds();

            int curPos = launcher.getCurrentPosition();
            int dPos = curPos - lastPos;

            double ticksPerSec = dPos / dt;
            double ticksPerRev = 28.0;  // change this to the correct value
            double rpm = Math.abs((ticksPerSec / ticksPerRev) * 60.0);

// Correct RPM formula

            telemetry.addData("RPM", rpm);
            telemetry.update();

            lastPos = curPos;
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


            if (llResult != null && llResult.isValid()) {

                Pose3D botPose = llResult.getBotpose_MT2();
                // Get the horizontal offset from the crosshair in degrees (-27 to 27)
                double tx = llResult.getTx(); // FTC

                // Calculate the turning speed using proportional control
                // A higher tx means the target is to the right, so we need to turn right (positive speed)
                turnSpeed = tx * Kp;
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

                // Clamp the turn speed to safe limits (e.g., -0.5 to 0.5)
                turnSpeed = Math.max(Math.min(turnSpeed, 0.5), -0.5);


                //Add telemetry/logging for tuning
                telemetry.addData("Target Found", true);
                telemetry.addData("TX", tx);
                telemetry.addData("turn Speed", turnSpeed);
                telemetry.update();

            } else {
                // No target found, stop turning
                turnSpeed = 0;
                HeadingLight.setPosition(0.0);
                telemetry.addData("Target Found", false);
                telemetry.update();
            }

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

//Code for Auto adjust angle (GamePad1 right_bumper)
            if (gamepad1.right_bumper)  //Auto heading to Target
            {
                double targetAngleError = llResult.getTx(); // Horizontal offset from Limelight

                // PID variables
                double integral = 0;
                double lastError = 0;

                // Simple PID control for rotation
                double proportional = Kp * targetAngleError;
                integral += Ki * targetAngleError;
                double derivative = Kd * (targetAngleError - lastError);
                lastError = targetAngleError;

                double turnPower = proportional + integral + derivative;

                // If error is small enough, stop turning
                if (Math.abs(targetAngleError) < 1.0) {
                    turnPower = 0;
                }
                telemetry.addData("turn power", turnPower);
                telemetry.update();
                // Apply differential power to rotate robot

                 bL.setPower(turnPower*20);
                 fL.setPower(turnPower*20);
                 bR.setPower(-turnPower*20);
                 fR.setPower(-turnPower*20);

            }

//Code for Gates

            if (gamepad2.right_stick_button)   // Open the Gate
            {
                Rgate.setPosition(0.18);
            }

            if (gamepad2.left_stick_button)   // Close the Gate
            {
                Rgate.setPosition(0);
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
                Canopy.setPower(-40);   // Turn on canopy
            }

            if (gamepad2.cross)   // canopy wheel turn off
            {
                Canopy.setPower(0);  // Turn off canopy
            }
// Launcher speed increase
            if (gamepad2.circle && !lastAState)   // Launcher speed increment
            {
                launcherspeed = launcher.getPower() + POWER_INCREMENT;
                if (launcherspeed > MAX_POWER)  launcherspeed = MAX_POWER;
                // Apply power to motor
                launcher.setPower(launcherspeed);
                sleep(100);
            }
            if (gamepad2.square && !lastBState)   // Launcher speed decrease
            {
                launcherspeed = launcher.getPower() - POWER_INCREMENT;
                if (launcherspeed < MIN_POWER) launcherspeed = MIN_POWER;
                launcher.setPower(launcherspeed);
                sleep(100);
            }

            // Update telemetry
            telemetry.addData("Motor Power", "%.2f", launcherspeed);
            telemetry.update();

            // Save button states for next loop
            lastAState = gamepad2.circle;
            lastBState = gamepad2.square;

// Sorter Code

            if (gamepad1.square)   // Sorter Left to Grab Purple Balls
            {
//                Sorter.setPosition(0.15);
                Sorter.setPosition(0.10);
            }
            if (gamepad1.circle)   // Sorter Right to Grab Green Balls
            {
//                Sorter.setPosition(0.62);
                Sorter.setPosition(0.7);
            }

            if (gamepad2.dpad_up)
            {
                launcher.setPower(0.75);
            }

            if (gamepad2.dpad_down)
            {
                launcher.setPower(0.55);
            }

            if (gamepad2.dpad_left)
            {
                launcher.setPower(0.6);
            }

            if (gamepad2.dpad_right)
            {
                launcher.setPower(0.85);
            }
            if (gamepad2.right_bumper)
            {
                Rlift.setPosition(0.24);
                sleep(200);
                Rlift.setPosition(0.05);
            }
            if (gamepad2.left_bumper)
            {
                Llift.setPosition(0.01);
                sleep(200);
                Llift.setPosition(0.2) ;
            }
            if (gamepad1.dpad_up)
            {
                lifter.setPower(1);
                lifter2.setPower(1);
            }
            else
            {
                lifter.setPower(0);
                lifter2.setPower(0);
            }
            if (gamepad1.dpad_down)
            {
                lifter.setPower(-1);
                lifter2.setPower(-1);
            }
            else
            {
                lifter.setPower(0);
                lifter2.setPower(0);
            }

            // Drive Wheel calculations

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x*1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double frontLeftPower  = ((y + x + rx)/denominator);
            double frontRightPower = ((y - x - rx)/denominator);
            double backLeftPower   =  ((y - x + rx)/denominator);
            double backRightPower  = ((y + x - rx)/denominator);

            fL.setPower(frontLeftPower);
            fR.setPower(frontRightPower);
            bL.setPower(backLeftPower);
            bR.setPower(backRightPower);
        }

    }
}