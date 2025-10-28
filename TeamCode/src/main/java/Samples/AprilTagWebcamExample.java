package Samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class AprilTagWebcamExample extends OpMode{
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    @Override
    public void init(){
        aprilTagWebcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop(){
        //update the vision portal
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
        telemetry.addData("id20 String", id20.toString());
    }
}