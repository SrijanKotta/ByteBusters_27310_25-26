package Samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name="FirstTeleOp", group="Linear OpMode")
//@Disabled
public class FirstTeleOpSample extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
    DcMotor fL = hardwareMap.dcMotor.get("front_left_drive");
    DcMotor fR = hardwareMap.dcMotor.get("front_right_drive");
    DcMotor bL = hardwareMap.dcMotor.get("back_left_drive");
    //DcMotor bR = hardwareMap.dcMotor.get("launcher");
    DcMotor bR = hardwareMap.dcMotor.get("back_right_drive");
    CRServo AxonServo = hardwareMap.get(CRServo.class, "AxonServo");


    // reverse the left side motors because some meccanum wheels are backwards
    //fL.setDirection(DcMotorSimple.Direction.REVERSE);
    bL.setDirection(DcMotorSimple.Direction.REVERSE);
    fL.setDirection(DcMotorSimple.Direction.REVERSE);
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
while (opModeIsActive()){
    int Lcount = 0;
    int Icount = 0;

    if (gamepad1.dpad_up)
    {
        if (Icount == 0)
        {
            AxonServo.setPower(100);
            Icount++;
        }
        if (Icount == 1)
        {
            AxonServo.setPower(100);
            Icount--;
        }

    }
    if (gamepad1.dpad_down)
    {
        if (Lcount == 0)
        {
            AxonServo.setPower(-100);
            Icount++;
        }
        if (Lcount == 1)
        {
            AxonServo.setPower(-100);
            Icount--;
        }
    }


//    if (gamepad1.dpad_up)
//    {
//        if (Lcount == 0)
//        {
//            //launcher.setPower(0)
//            Lcount++;
//        }
//        if (Lcount == 1)
//        {
//            //launcher.setPower(0)
//            Lcount--;
//
//        }
//    }

    double y = -gamepad1.left_stick_y;
    double x = gamepad1.left_stick_x*1.1;
    double rx = gamepad1.right_stick_x;

    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

    double frontLeftPower  = ((y + x + rx)/denominator);
    double frontRightPower = ((y - x - rx)/denominator);
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