package org.firstinspires.ftc.teamcode;
import android.util.Size;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
    private DcMotor frontLeft, backLeft, frontRight, backRight, intake;
    private DcMotorEx spinner;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        spinner = hardwareMap.get(DcMotorEx.class, "spinner");
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

        double spinnerP=18, spinnerI=0, spinnerD=0, spinnerF=16;
        PIDFCoefficients pidfSpinner = new PIDFCoefficients(spinnerP, spinnerI, spinnerD, spinnerF);
        spinner.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfSpinner);

        double direction, turn, speed;//for the movement of the robot

        int powerLevel=0;
        double spinnerTimer=0;

        double[] spinnerPower = {0,1400,1540,1680,1820,1960,2100,2240};

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //initAprilTag();
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
                } else if (this.gamepad2.right_bumper && powerLevel < spinnerPower.length-1 && spinnerTimer==0) {
                    powerLevel++;
                    spinnerTimer=getRuntime();
                }
                if (getRuntime()-spinnerTimer>=0.4){
                    spinnerTimer=0;
                }

                spinner.setVelocity(spinnerPower[powerLevel]);

                direction = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x);
                turn = -gamepad1.right_stick_x;
                speed = Math.sqrt((gamepad1.left_stick_y * gamepad1.left_stick_y) + (gamepad1.left_stick_x * gamepad1.left_stick_x));
                frontLeft.setPower(Math.sin(direction + (0.25 * (Math.PI))) * speed + turn);
                frontRight.setPower(Math.sin(direction - (0.25 * (Math.PI))) * speed - turn);
                backLeft.setPower(Math.sin(direction - (0.25 * (Math.PI))) * speed + turn);
                backRight.setPower(Math.sin(direction + (0.25 * (Math.PI))) * speed - turn);

                //adds telemetry data
                telemetry.addData("Power Level", powerLevel);
                telemetry.addData("Spinner Velocity", spinner.getVelocity()/28*60);
                telemetry.addData("Spinner Target Velocity", spinnerPower[powerLevel]/28*60);
                telemetry.update();
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