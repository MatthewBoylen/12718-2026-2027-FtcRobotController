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

    @Override
    public void runOpMode() {
        spinner = hardwareMap.get(DcMotorEx.class, "spinner");
        spinner.setDirection(DcMotor.Direction.REVERSE);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double spinnerP = 0, spinnerI = 0, spinnerD = 0, spinnerF = 14;
        PIDFCoefficients pidfSpinner = new PIDFCoefficients(spinnerP, spinnerI, spinnerD, spinnerF);
        spinner.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfSpinner);


        double spinnerTimer = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();


        if (opModeIsActive()) {
            while (opModeIsActive()) {
                spinner.setVelocity(1680);
                telemetry.addData("Speed", spinner.getVelocity());
                telemetry.addData("Power", spinner.getPower());
                telemetry.update();
            }
        }
    }
}