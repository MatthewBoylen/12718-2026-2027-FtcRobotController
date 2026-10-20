//This is for red
package org.firstinspires.ftc.teamcode;
import android.graphics.Color;
import android.util.Size;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
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
import org.firstinspires.ftc.teamcode.pedro.Tuning;
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
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.nio.file.Paths;
import java.util.List;
import com.qualcomm.robotcore.hardware.PwmControl;
@Autonomous
public class RedAuto1 extends LinearOpMode {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private ColorBlobLocatorProcessor colorLocatorRed;
    private ColorBlobLocatorProcessor colorLocatorBlue;
    private ColorBlobLocatorProcessor colorLocatorPollen;
    private double distanceFromBasket, distanceFromCenterOfBasket;
    private boolean basket1Active, basket2Active;
    private DcMotor intake, launcher;
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();


    //Poses
    private final Pose startPose = p.of(24, 24, 0);
    private final Pose score1Pose = p.of(48, 48, 90);
    private final Pose flower1Pose = p.of(48, 48, 90);

    //Paths
    private Path startToScore1() {return line(startPose, score1Pose).linear(startPose, score1Pose);}
    private Path score1ToFlower1() {return line(score1Pose, flower1Pose).linear(score1Pose, flower1Pose);}

    private Command autoRoutine() {
        return sequential(
                follow(follower, startToScore1()),
                // Add mechanism commands here.
                follow(follower, score1ToFlower1())
        );
    }


    public void runOpMode() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotor.class, "spinner");
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        initAprilTagAndColorBlob();
        waitForStart();

        if (opModeIsActive()){
            schedule(autoRoutine());
            while (opModeIsActive()) {
                follower.update();
                Scheduler.execute();

            }
        }
        if (visionPortal!=null){
            visionPortal.close();
        }//end processes
        sleep(50);
    }
    private void initAprilTagAndColorBlob(){
        aprilTag = new AprilTagProcessor.Builder().setDrawCubeProjection(true).build();
        colorLocatorRed = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)// Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image
                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();
        colorLocatorBlue = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)// Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image
                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();
        colorLocatorPollen = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)// Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image
                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "camera"));
        builder.setCameraResolution(new Size(640, 480));
        sleep(20);
        builder.addProcessor(aprilTag);
        builder.addProcessor(colorLocatorRed);
        builder.addProcessor(colorLocatorBlue);
        builder.addProcessor(colorLocatorPollen);
        builder.enableLiveView(true);
        VisionPortal visionPortal = builder.build();
        visionPortal.setProcessorEnabled(aprilTag, true);
        sleep(100);
    }
    private void sortAndShowBlobs(){
        List<ColorBlobLocatorProcessor.Blob> redBlobs = colorLocatorRed.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 50, 20000, redBlobs);  // filter out very small blobs.
        ColorBlobLocatorProcessor.Util.filterByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.6, 1, redBlobs);

        List<ColorBlobLocatorProcessor.Blob> blueBlobs = colorLocatorBlue.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 50, 20000, blueBlobs);  // filter out very small blobs.
        ColorBlobLocatorProcessor.Util.filterByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.6, 1, blueBlobs);

        List<ColorBlobLocatorProcessor.Blob> pollenBlobs = colorLocatorPollen.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 50, 20000, pollenBlobs);  // filter out very small blobs.
        ColorBlobLocatorProcessor.Util.filterByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.6, 1, pollenBlobs);


    }
    private void getActiveBasket(){
        List<AprilTagDetection> ad = aprilTag.getDetections();
        int basket1Height = 0;
        int basket1NumberOfDetections = 0;
        int basket2Height = 0;
        int basket2NumberOfDetections = 0;
        for (AprilTagDetection tag :ad){
        //get the correct april tag numbers and make sure the equals thing is right
            if (tag.equals(30)||tag.equals(31)||tag.equals(32)){
                basket1Height+=tag.ftcPose.y;
                basket1NumberOfDetections++;
            }
            else if (tag.equals(30)||tag.equals(31)||tag.equals(32)){
                basket2Height+=tag.ftcPose.y;
                basket2NumberOfDetections++;
            }
        }
        if (basket1Height/basket1NumberOfDetections>basket2Height/basket2NumberOfDetections){
            basket1Active=true;
            basket2Active=false;
        }
        else if (basket1Height/basket1NumberOfDetections<basket2Height/basket2NumberOfDetections){
            basket1Active=false;
            basket2Active=true;
        }
    }
    private void launchBalls(){

    }
}
