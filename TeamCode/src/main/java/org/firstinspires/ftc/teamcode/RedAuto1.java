//This is for red
package org.firstinspires.ftc.teamcode;
import android.util.Size;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagCanvasAnnotator;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import java.nio.file.Paths;
import java.util.List;
import com.qualcomm.robotcore.hardware.PwmControl;
@Autonomous
public class RedAuto1 extends LinearOpMode {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private double distanceFromBasket, distanceFromCenterOfBasket;
    private boolean basket1Tipped, basket2Tipped;

    public void runOpMode() {
        initAprilTag();
        waitForStart();

        if (opModeIsActive()){
            while (opModeIsActive()) {

            }
        }
        if (visionPortal!=null){
            visionPortal.close();
        }//end processes
        sleep(50);
    }
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawCubeProjection(true)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "camera"));
        builder.setCameraResolution(new Size(640, 480));
        sleep(20);
        builder.addProcessor(aprilTag);
        builder.enableLiveView(true);
        VisionPortal visionPortal = builder.build();
        visionPortal.setProcessorEnabled(aprilTag, true);
    }
    private void getBasket(){
        List<AprilTagDetection> ad = aprilTag.getDetections();
        for(AprilTagDetection tag : ad){

        }
    }
}
