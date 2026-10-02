package org.firstinspires.ftc.teamcode;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name="pidfTest", group="test")
public class pidfTest extends LinearOpMode {
    private DcMotorEx spinner;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {
        spinner = hardwareMap.get(DcMotorEx.class, "spinner");
        spinner.setDirection(DcMotor.Direction.REVERSE);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double spinnerP = 32767.0/2400.0, spinnerI = 1, spinnerD = 1, spinnerF = 0;
        PIDFCoefficients pidfSpinner = new PIDFCoefficients(spinnerP, spinnerI, spinnerD, spinnerF);
        spinner.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfSpinner);

        double direction, turn, speed;//for the movement of the robot

        int powerLevel = 0;
        double spinnerTimer = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();


        if (opModeIsActive()) {
            while (opModeIsActive()) {
                spinner.setVelocity(2800);
                telemetry.addData("Speed", spinner.getVelocity());
                telemetry.update();
            }
        }
    }
}