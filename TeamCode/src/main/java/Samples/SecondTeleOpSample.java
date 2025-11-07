package Samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="SecondTeleOp", group="Linear OpMode")
//@Disabled
public class SecondTeleOpSample extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
    DcMotor fL = hardwareMap.dcMotor.get("front_left_drive");
    DcMotor fR = hardwareMap.dcMotor.get("front_right_drive");
    DcMotor bL = hardwareMap.dcMotor.get("back_left_drive");
    DcMotor launcher = hardwareMap.dcMotor.get("launcher");
    DcMotor bR = hardwareMap.dcMotor.get("back_right_drive");
    CRServo AxonServo = hardwareMap.get(CRServo.class, "AxonServo");
    CRServo Canopy = hardwareMap.get(CRServo.class, "Canopy");
    Servo Rlift = hardwareMap.get(Servo.class, "Rlift");
    Servo Llift = hardwareMap.get(Servo.class, "Llift");
    Servo Sorter = hardwareMap.get(Servo.class, "Sorter");


    // reverse the left side motors because some meccanum wheels are backwards
    //fL.setDirection(DcMotorSimple.Direction.REVERSE);
    bL.setDirection(DcMotorSimple.Direction.REVERSE);
    fL.setDirection(DcMotorSimple.Direction.FORWARD);
    fR.setDirection(DcMotorSimple.Direction.FORWARD);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
  // The code after this is only run after start is pressed
        waitForStart();
        runtime.reset();

        if (isStopRequested()){
            return;
        }
        //This is loop that checks the gamepad for inputs every iteration
while (opModeIsActive())
{
    double max;
    double speed = 0.6;
    int i = 0;
    int j = 0;
    int k = 0;

    Canopy.setPower(40);

    if (gamepad2.dpad_up)
    {
        launcher.setPower(-1);
    }
    if (gamepad2.dpad_down)
    {
        launcher.setPower(0);
    }


    if (gamepad1.a)
    {
        if (i == 0)
        {
            AxonServo.setPower(100);
            Canopy.setPower(40);
            i = 1;
        }
        else {
            AxonServo.setPower(0);
            Canopy.setPower(0);
            i = 0;
        }
    }

    if (gamepad1.b)
    {
        if (j == 0)
        {
            AxonServo.setPower(-100);
            j = 1;
        }
        else {
            AxonServo.setPower(0);
            j = 0;
        }
    }

    if (gamepad1.x)
    {
        if (k == 0)
        {
            launcher.setPower(-1);
            Canopy.setPower(40);
            k = 1;
        }
        else {
            launcher.setPower(0);
            Canopy.setPower(0);
            k = 0;
        }
    }

    if (gamepad1.dpad_up)
    {
        AxonServo.setPower(100);

    }
    if (gamepad1.dpad_down)
    {
        AxonServo.setPower(-100);

    }

    if (gamepad1.dpad_right)
    {
        Sorter.setPosition(0.6);

    }
   if (gamepad1.dpad_left)
   {
       Sorter.setPosition(0.1);
   }

    if (gamepad2.right_bumper)
    {
        Rlift.setPosition(0.92);
        sleep(1000);
        Rlift.setPosition(1);
    }
    if (gamepad2.left_bumper)
    {
        Llift.setPosition(0.2);
        sleep(1000);
        Llift.setPosition(0);
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

    double y = -gamepad1.left_stick_y;
    double x = gamepad1.left_stick_x*1.1;
    double rx = gamepad1.right_stick_x;

    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

    double frontLeftPower  = ((y + x + rx)/denominator)/1.25;
    double frontRightPower = ((y - x - rx)/denominator)/1.25;
    double backLeftPower   =  ((y - x + rx)/denominator);
    double backRightPower  = ((y + x - rx)/denominator);



//    double frontLeftPower  = 0.2;
//    double frontRightPower = 0.2;
//    double backLeftPower   =  0.2;
//    double backRightPower  = 0.2;

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