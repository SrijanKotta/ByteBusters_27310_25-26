package TeleOpCode;

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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import android.view.KeyEvent;
import android.view.InputDevice;


@TeleOp(name="MainTeleop_Test1", group="Linear OpMode")
//@Disabled
public class MainTeleop_Test1 extends LinearOpMode {



    //Define the Limelight Object
    private Limelight3A limelight;


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
    private IMU imu;
    private double launcherspeed;
    // PID Constants
    private final double POWER_INCREMENT = 0.05; // Increase by 5% each press
    private final double MAX_POWER = 1.0; // 100% power
    private final double MIN_POWER = -1.0; // Reverse full power
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


            if (llResult != null && llResult.isValid()) {

                Pose3D botPose = llResult.getBotpose_MT2();
                // Get the horizontal offset from the crosshair in degrees (-27 to 27)
                double tx = llResult.getTx(); // FTC


                // A higher tx means the target is to the right, so we need to turn right (positive speed)
                if (tx > -1.0 && tx < 1.0)
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

                double maxTurnPower = Math.min(Math.abs(turnPower * 10), 1.0); // Ensure max power = 1.0

//                 bL.setPower(turnPower*20);
//                 fL.setPower(turnPower*20);
//                 bR.setPower(-turnPower*20);
//                 fR.setPower(-turnPower*20);

                bL.setPower(maxTurnPower);
                fL.setPower(maxTurnPower);
                bR.setPower(-maxTurnPower);
                fR.setPower(-maxTurnPower);

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

//                sleep(100); //Commented to avoid sleeps in the code
            }
            if (gamepad2.square && !lastBState)   // Launcher speed decrease
            {
                launcherspeed = launcher.getPower() - POWER_INCREMENT;
                if (launcherspeed < MIN_POWER) launcherspeed = MIN_POWER;
                launcher.setPower(launcherspeed);
//                sleep(100); //Commented to avoid sleeps in the code
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