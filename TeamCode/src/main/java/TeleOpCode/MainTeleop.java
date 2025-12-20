package TeleOpCode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
//Color sensor libraries

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import android.graphics.Color;

@TeleOp(name="MainTeleOp", group="Linear OpMode")
//@Disabled
public class MainTeleop extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private NormalizedColorSensor colorSensor = null;

//    private final float GREEN_HUE_MIN = 80;
//    private final float GREEN_HUE_MAX = 145;
//    private ElapsedTime pidTimer = new ElapsedTime();


//    ColorDetection cd = new ColorDetection();
//    ColorDetection.DetectedColor detectedColor;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor fL = hardwareMap.dcMotor.get("front_left_drive");
        DcMotor fR = hardwareMap.dcMotor.get("front_right_drive");
        DcMotor bL = hardwareMap.dcMotor.get("back_left_drive");
        DcMotor launcher = hardwareMap.dcMotor.get("launcher");
        DcMotor lifter = hardwareMap.dcMotor.get("Lifter");
        DcMotor bR = hardwareMap.dcMotor.get("back_right_drive");
        DcMotor grabber = hardwareMap.dcMotor.get("grabber");
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
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

//        cd.init(hardwareMap);

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
                        grabber.setPower(0);   // Turn off Intake
                        IntakeWheel = false;
                    }
                    else
                    {
                        grabber.setPower(-1);  // Turn on Intake
                        IntakeWheel = true;
                    }

                }

            if (gamepad1.cross)   // Intake wheel reverse
            {
                if (IntakeWheel)
                {
                    grabber.setPower(0);   // Turn off Intake
                    IntakeWheel = false;
                }
                else
                {
                    grabber.setPower(1);  // Turn on Intake
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
                Sorter.setPosition(0.15);
            }
            if (gamepad1.circle)   // Sorter Right
            {
                Sorter.setPosition(0.62);
            }

            if (gamepad2.dpad_up)
            {
                launcher.setPower(0.7);
//                Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//                sleep(400);
//                Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
//                sleep(400);
//                Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

            }

            if (gamepad2.dpad_down)
            {
                launcher.setPower(0.5);
//                Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//                sleep(400);
//                Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
//                sleep(400);
//                Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }

            if (gamepad2.dpad_left)
            {
                launcher.setPower(0.6);
//                Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//                sleep(400);
//                Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
//                sleep(400);
//                Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }

            if (gamepad2.dpad_right)
            {
                launcher.setPower(0.8);
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
                Rlift.setPosition(0.24);
                sleep(200);
                Rlift.setPosition(0.05);
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