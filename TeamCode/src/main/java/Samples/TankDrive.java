package Samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name="Tank Drive", group="Linear OpMode")
//@Disabled
public class
TankDrive extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        CRServo AxonServo = hardwareMap.get(CRServo.class, "AxonServo");
        CRServo Canopy = hardwareMap.get(CRServo.class, "Canopy");
        Servo Rlift = hardwareMap.get(Servo.class, "Rlift");
        Servo Llift = hardwareMap.get(Servo.class, "Llift");
        Servo Sorter = hardwareMap.get(Servo.class, "Sorter");


        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        double frontLeftPower  = 0;
        double frontRightPower = 0;
        double backLeftPower   = 0;
        double backRightPower  = 0;
        double speed = 0.6;

        while (opModeIsActive()) {
            double max;
            int Lcount = 0;
            int Icount = 0;

            Canopy.setPower(100);





            if (gamepad1.dpad_right)
            {
                if (Icount == 0)
                {
                    AxonServo.setPower(100);
                    Icount++;
                }
                else if (Icount == 1)
                {
                    AxonServo.setPower(0);
                    Icount--;
                }

            }
            else if (gamepad1.dpad_left)
            {
                if (Lcount == 0)
                {
                    AxonServo.setPower(-100);
                    Icount++;
                }
                else if (Lcount == 1)
                {
                    AxonServo.setPower(0);
                    Icount--;
                }
            }
            
            if (gamepad2.dpad_right)
            {
                Sorter.setPosition(0.4);
            }
            else if (gamepad2.dpad_left)
            {
                Sorter.setPosition(0.25);
            }

            else if (gamepad1.dpad_up)
            {
                frontLeftPower  = speed;
                frontRightPower = speed;
                backLeftPower   = speed;
                backRightPower  = speed;
            }
            else if (gamepad1.dpad_down)
            {
                frontLeftPower  = -speed;
                frontRightPower = -speed;
                backLeftPower   = -speed;
                backRightPower  = -speed;
            }

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);
//            else
//            {
//                frontLeftPower  = 0;
//                frontRightPower = 0;
//                backLeftPower   = 0;
//                backRightPower  = 0;
//            }
//            else if (gamepad1.dpad_right)
//            {
//                speed = 0.8;
//            }
//            else if (gamepad1.dpad_left)
//            {
//                speed = 0.1;
//            }
            if (gamepad1.x)
            {
                Rlift.setPosition(0.7);
                sleep(1000);
                Rlift.setPosition(0.5);
            }
            if (gamepad1.b)
            {
                Llift.setPosition(0);
                sleep(1000);
                Llift.setPosition(0.2);
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



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.update();
        }
    }}
