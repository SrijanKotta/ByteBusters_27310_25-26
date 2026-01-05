package TeleOpCode;


import android.graphics.Color;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="MainTeleOp_Test", group="Linear OpMode")
//@Disabled
public class MainTeleop_Test extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private NormalizedColorSensor colorSensor = null;

    private final float GREEN_HUE_MIN = 80;
    private final float GREEN_HUE_MAX = 145;

    private IMU imu;
    // PID Constants
    private static final double Kp = 0.001;
    private static final double Ki = 0.0002;
    private static final double Kd = 0.00005;

    // Your measured max velocity at full power
    private static final double MAX_VELOCITY = 2000;

    // Shooter speed (adjustable from 0.0 to 1.0)
    private double shooterSpeed = 0.5;

    // Target velocities
    private double targetVelocity1 = 0;

    // PID variables
    private double integralSum1 = 0;
    private double lastError1 = 0;


    // Shooter management
    private boolean shootersOn = true;  // CHANGED: start with shooters ON
    private boolean prevRT1 = false;
    private boolean prevLT1 = false;

    // Gamepad 2 controls
    private boolean prevx2 = false;
    private boolean prevB2 = false;
    private static final double TRIGGER_THRESH = 0.2;
    private ElapsedTime pidTimer = new ElapsedTime();


//    ColorDetection cd = new ColorDetection();
//    ColorDetection.DetectedColor detectedColor;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor fL = hardwareMap.dcMotor.get("front_left_drive");
        DcMotor fR = hardwareMap.dcMotor.get("front_right_drive");
        DcMotor bL = hardwareMap.dcMotor.get("back_left_drive");
//        DcMotorEX launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        DcMotor lifter = hardwareMap.dcMotor.get("Lifter");
        DcMotor bR = hardwareMap.dcMotor.get("back_right_drive");
        CRServo AxonServo = hardwareMap.get(CRServo.class, "AxonServo");
        CRServo Canopy = hardwareMap.get(CRServo.class, "Canopy");
        Servo Rlift = hardwareMap.get(Servo.class, "Rlift");
        Servo Llift = hardwareMap.get(Servo.class, "Llift");
        Servo Sorter = hardwareMap.get(Servo.class, "Sorter");
        RevBlinkinLedDriver Light = hardwareMap.get(RevBlinkinLedDriver.class,"LED_light");

        // reverse the left side motors because some
        //meccanum wheels are backwards
        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.FORWARD);
        fL.setDirection(DcMotorSimple.Direction.FORWARD);
        fR.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

//        cd.init(hardwareMap);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Shooters use encoders for PID velocity control
        launcher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pidTimer.reset();

        // CHANGED: Initialize shooters to start spinning immediately
        targetVelocity1 = MAX_VELOCITY * shooterSpeed;
        integralSum1 = 0;
        lastError1 = 0;

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
            double ticksPerRev = 537.6;  // change this to the correct value
            double rpm = Math.abs((ticksPerSec / ticksPerRev) * 60.0);

// Correct RPM formula

            telemetry.addData("RPM", rpm);
            telemetry.update();

            lastPos = curPos;
            timer.reset();

            telemetry.addData("RPM", rpm);
            telemetry.update();

            lastPos = curPos;
            timer.reset();
            //Canopy.setPower(40);


    /*
GamePad 1
    Triangle - Intake - backwards
    X - intake - inwards
    Square - Sorter Left
    Circle - Sorter Right
 */
/*
Color sensor code to get the values
*/


     // Get color sensor data

            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            //NormalizedColorSensor.NormalizedRGBA colors = colorSensor.getNormalizedColors();
            float[] hsvValues = new float[3];
            Color.colorToHSV(colors.toColor(), hsvValues);
/*
            detectedColor = cd.getDetectedColor(telemetry);
            telemetry.addData("Color Detected", detectedColor);
*/
            // Code for Intake
            if (gamepad2.cross)


                if (gamepad1.triangle)   // Intake Wheel Inward
                {
                    if (IntakeWheel)
                    {
                        AxonServo.setPower(0);   // Turn off Intake
                        IntakeWheel = false;
                    }
                    else
                    {
                        AxonServo.setPower(100);  // Turn on Intake
                        IntakeWheel = true;
                    }

                }

            if (gamepad1.cross)   // Intake wheel reverse
            {
                if (IntakeWheel)
                {
                    AxonServo.setPower(0);   // Turn off Intake
                    IntakeWheel = false;
                }
                else
                {
                    AxonServo.setPower(-100);  // Turn on Intake
                    IntakeWheel = true;
                }
            }
// canopy code
            if (gamepad2.triangle)   // canopy Wheel Inward
            {
                if (TriCanopy)
                {
                    Canopy.setPower(0);   // Turn off canopy
                    TriCanopy = false;
                }
                else
                {
                    Canopy.setPower(40);;  // Turn on canopy
                    TriCanopy = true;
                }

            }

            if (gamepad2.cross)   // canopy wheel reverse
            {
                if (CrossCanopy)
                {
                    Canopy.setPower(0);   // Turn off canopy
                    CrossCanopy = false;
                }
                else
                {
                    Canopy.setPower(-40);  // Turn on canopy
                    CrossCanopy = true;
                }
            }
            // Sorter Code

/*
            if (hsvValues[0] >= GREEN_HUE_MIN && hsvValues[0] <= GREEN_HUE_MAX)
            {
                Sorter.setPosition(0.6);
            }
            else
            {
                Sorter.setPosition(0.1);
            }
*/

/*

            if (detectedColor == "GREEN")
            {
                Sorter.setPosition(0.6);
            }
            else if (detectedColor == "PURPLE")
            {
                Sorter.setPosition(0.1);
            }

 */

            if (gamepad1.square)   // Sorter Left
            {
                Sorter.setPosition(0.1);
            }
            if (gamepad1.circle)   // Sorter Right
            {
                Sorter.setPosition(0.6);
            }
//Shooter PID Control Code Starts
            // =======================
            // Shooters (gamepad2 triggers)
            // =======================
            boolean rt1 = gamepad2.right_trigger > TRIGGER_THRESH;
            boolean lt1 = gamepad2.left_trigger > TRIGGER_THRESH;

            // Start shooters with right trigger
            if (rt1 && !prevRT1) {
                shootersOn = true;
                targetVelocity1 = MAX_VELOCITY * shooterSpeed;
                integralSum1 = 0;
                lastError1 = 0;
                pidTimer.reset();
            }

            // Stop shooters with left trigger
            if (lt1 && !prevLT1) {
                shootersOn = false;
                launcher.setPower(0);
                targetVelocity1 = 0;
            }
            prevRT1 = rt1;
            prevLT1 = lt1;

            // =======================
            // Speed Adjustment (gamepad2)
            // =======================
            boolean x2 = gamepad2.x;
            boolean b2 = gamepad2.b;

            // Increase shooter speed by 0.01
            if (x2 && !prevx2) {
                shooterSpeed = Math.min(1.0, shooterSpeed + 0.01);
                if (shootersOn) {
                    targetVelocity1 = MAX_VELOCITY * shooterSpeed;
                }
            }

            // Decrease shooter speed by 0.01
            if (b2 && !prevB2) {
                shooterSpeed = Math.max(0.0, shooterSpeed - 0.01);
                if (shootersOn) {
                    targetVelocity1 = MAX_VELOCITY * shooterSpeed;
                }
            }

            prevx2 = x2;
            prevB2 = b2;

            // =======================
            // PID Control Loop
            // =======================
            double power1 = 0, power2 = 0;
            double vel1 = 0, vel2 = 0;
            double error1 = 0, error2 = 0;

            if (shootersOn && targetVelocity1 > 0) {
                vel1 = Math.abs(launcher.getVelocity());

                double dt_p = pidTimer.seconds();
                pidTimer.reset();

                if (dt_p == 0 || dt_p > 0.1) dt_p = 0.02;

                // SHOOTER 1 PID
                error1 = targetVelocity1 - vel1;
                double derivative1 = (error1 - lastError1) / dt;
                integralSum1 += error1 * dt;
                integralSum1 = Math.max(-500, Math.min(500, integralSum1));

                power1 = (Kp * error1) + (Ki * integralSum1) + (Kd * derivative1);
                power1 = Math.max(0, Math.min(1.0, power1));

                lastError1 = error1;
                launcher.setPower(power1);
            }
//Shooter PID Control Code Ends
            if (gamepad2.dpad_up)
            {
                //launcher.setPower(0.7);
                shooterSpeed = 0.7;
                targetVelocity1 = MAX_VELOCITY * shooterSpeed;
//                Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//                sleep(400);
//                Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
//                sleep(400);
//                Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

            }

            if (gamepad2.dpad_down)
            {
                //launcher.setPower(0.5);
                shooterSpeed = 0.5;
                targetVelocity1 = MAX_VELOCITY * shooterSpeed;
//                Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//                sleep(400);
//                Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
//                sleep(400);
//                Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }

            if (gamepad2.dpad_left)
            {
                //launcher.setPower(0.6);
                shooterSpeed = 0.6;
                targetVelocity1 = MAX_VELOCITY * shooterSpeed;
//                Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//                sleep(400);
//                Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
//                sleep(400);
//                Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }

            if (gamepad2.dpad_right)
            {
                //launcher.setPower(0.8);
                shooterSpeed = 0.8;
                targetVelocity1 = MAX_VELOCITY * shooterSpeed;
//                Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//                sleep(400);
//                Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
//                sleep(400);
//                Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
/*
            if (gamepad1.dpad_right)
            {
                Sorter.setPosition(0.6);
            }

            if (gamepad1.dpad_left)
            {
                Sorter.setPosition(0.1);
            }
*/
            if (gamepad2.right_bumper)
            {
                Rlift.setPosition(.5);
                sleep(200);
                Rlift.setPosition(0);
                // Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }
            if (gamepad2.left_bumper)
            {
                Llift.setPosition(0.01);
                sleep(200);
                Llift.setPosition(0.2) ;
                // Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }

            if (gamepad1.a)
            {
                speed = speed + 0.05;
                sleep(1000);
            }
            if (gamepad1.y)
            {
                speed = speed - 0.05;
                sleep(1000);
            }
            if (gamepad1.right_bumper)
            {
                lifter.setPower(-1);

            }
            if (gamepad1.left_bumper)
            {
                lifter.setPower(1);

            }

            // Drive Wheel calculations

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x*1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double frontLeftPower  = ((y + x + rx)/denominator)/1;
            double frontRightPower = ((y - x - rx)/denominator)/1;
            double backLeftPower   =  ((y - x + rx)/denominator);
            double backRightPower  = ((y + x - rx)/denominator);




            fL.setPower(frontLeftPower);
            fR.setPower(frontRightPower);
            bL.setPower(backLeftPower);
            bR.setPower(backRightPower);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.update();
        }

    }
}