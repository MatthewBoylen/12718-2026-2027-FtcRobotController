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

        double spinnerP = 0, spinnerF = 0;
        PIDFCoefficients pidfSpinner = new PIDFCoefficients(spinnerP, 0, 0, spinnerF);
        spinner.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfSpinner);

        int TPR = 28;

        double high = 5000/60.0 * TPR;
        double low = 3000/60.0 * TPR;
        double curVel = 0;
        double[] step = {10,1,0.1,0.01, 0.001, 0.0001};
        int stepInex=1;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();


        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (gamepad1.leftBumperWasPressed()){
                    if(curVel==high){
                        curVel=low;
                    } else if (curVel==low) {
                        curVel=0;
                    }
                } else if (gamepad1.rightBumperWasPressed()) {
                    if(curVel==0){
                        curVel=low;
                    } else if (curVel==low) {
                        curVel=high;
                    }
                }

                if (gamepad1.bWasPressed()){
                    stepInex = (stepInex + 1)% step.length;
                }

                if (gamepad1.dpadLeftWasPressed()){
                    spinnerF -= step[stepInex];
                } else if (gamepad1.dpadRightWasPressed()) {
                    spinnerF += step[stepInex];
                }

                if (gamepad1.dpadDownWasPressed()){
                    spinnerP -= step[stepInex];
                } else if (gamepad1.dpadUpWasPressed()) {
                    spinnerP += step[stepInex];
                }

                pidfSpinner = new PIDFCoefficients(spinnerP, 0, 0, spinnerF);
                spinner.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfSpinner);

                spinner.setVelocity(curVel);

                double error = curVel - spinner.getVelocity();

                telemetry.addData("Speed", spinner.getVelocity()/TPR*60);
                telemetry.addData("Error", error/TPR*60);
                telemetry.addData("P", spinnerP);
                telemetry.addData("F", spinnerF);
                telemetry.addData("Step Size", step[stepInex]);
                telemetry.update();
            }
        }
    }
}