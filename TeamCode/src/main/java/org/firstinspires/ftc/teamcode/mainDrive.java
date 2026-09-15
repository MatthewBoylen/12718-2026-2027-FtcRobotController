package org.firstinspires.ftc.teamcode;
import android.util.Size;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name="mainDrive", group="mainDrive")
public class mainDrive extends LinearOpMode {
    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double direction, turn, speed;//for the movement of the robot

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        initAprilTag();
        waitForStart();

        if (opModeIsActive()){
            while (opModeIsActive()) {
                direction = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x);
                turn = -gamepad1.right_stick_x;
                speed = Math.sqrt((gamepad1.left_stick_y * gamepad1.left_stick_y) + (gamepad1.left_stick_x * gamepad1.left_stick_x));
                frontLeft.setPower(Math.sin(direction + (0.25 * (Math.PI))) * speed + turn);
                frontRight.setPower(Math.sin(direction - (0.25 * (Math.PI))) * speed - turn);
                backLeft.setPower(Math.sin(direction - (0.25 * (Math.PI))) * speed + turn);
                backRight.setPower(Math.sin(direction + (0.25 * (Math.PI))) * speed - turn);
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
    private void alignWithBasket(){

    }
}