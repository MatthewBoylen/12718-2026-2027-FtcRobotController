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
    private DcMotor frontLeft, backLeft, frontRight, backRight, spinner, intake;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        spinner = hardwareMap.get(DcMotor.class, "spinner");
        intake = hardwareMap.get(DcMotor.class, "intake");
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        spinner.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double direction, turn, speed;//for the movement of the robot

        int powerLevel=0;
        double spinnerTimer=0;

        double spinnerMax=0.8;
        double spinnerHighMax=0.75;
        double spinnerHigh=0.7;
        double spinnerMediumHigh=0.65;
        double spinnerMedium=0.6;
        double spinnerMediumLow=0.55;
        double spinnerLow=0.5;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        initAprilTag();
        waitForStart();


        if (opModeIsActive()){
            while (opModeIsActive()) {

                if(this.gamepad2.x){
                    intake.setPower(1);
                }
                else{
                    intake.setPower(-1);
                }


                if(this.gamepad2.left_bumper && powerLevel>0 && spinnerTimer==0){
                    powerLevel--;
                    spinnerTimer=getRuntime();
                } else if (this.gamepad2.right_bumper && powerLevel<7 && spinnerTimer==0) {
                    powerLevel++;
                    spinnerTimer=getRuntime();
                }
                if (getRuntime()-spinnerTimer>=0.4){
                    spinnerTimer=0;
                }

                if (powerLevel==0){
                    spinner.setPower(0);
                }else if (powerLevel==1){
                    spinner.setPower(spinnerLow);
                }else if(powerLevel ==2){
                    spinner.setPower(spinnerMediumLow);
                } else if (powerLevel==3){
                    spinner.setPower(spinnerMedium);
                }else if (powerLevel==4){
                    spinner.setPower(spinnerMediumHigh);
                }else if (powerLevel==5){
                    spinner.setPower(spinnerHigh);
                }else if (powerLevel==6){
                    spinner.setPower(spinnerHighMax);
                }else if (powerLevel==7){
                    spinner.setPower(spinnerMax);
                }

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

}