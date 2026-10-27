package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name="mainDrivePedro", group="Experemental")
public class mainDrivePedroPathing extends LinearOpMode {
    private DcMotor intake;
    private Follower follower;
    private DcMotorEx spinner;
    private ServoImplEx launcher;

    @Override
    public void runOpMode(){
        spinner = hardwareMap.get(DcMotorEx.class, "spinner");
        intake = hardwareMap.get(DcMotor.class, "intake");
        spinner.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcher = hardwareMap.get(ServoImplEx.class, "launcher");
        launcher.setPwmRange(new PwmControl.PwmRange(500, 2500));

        double spinnerP = 18, spinnerI = 0, spinnerD = 0, spinnerF = 15.5;
        PIDFCoefficients pidfSpinner = new PIDFCoefficients(spinnerP, spinnerI, spinnerD, spinnerF);
        spinner.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfSpinner);

        double scale = 0.8; // for the movement of the robot
        follower = Constants.create(hardwareMap);

        int powerLevel = 0;
        double spinnerTimer = 0;

        double[] spinnerPower = {0, 1400, 1540, 1680, 1820, 1960, 2100, 2240};

        double launchPollen = 0.69, launchNectar = 0.36;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        launcher.setPosition(launchNectar);

        while(opModeIsActive()){
            if (this.gamepad2.dpad_right) {
                intake.setPower(1);
            } else if (this.gamepad2.dpad_left) {
                intake.setPower(-1);
            } else if (this.gamepad2.dpad_down) {
                intake.setPower(0);
            }

            if (this.gamepad2.left_bumper && powerLevel > 0 && spinnerTimer == 0) {
                powerLevel--;
                spinnerTimer = getRuntime();
            } else if (this.gamepad2.right_bumper
                    && powerLevel < spinnerPower.length - 1
                    && spinnerTimer == 0) {
                powerLevel++;
                spinnerTimer = getRuntime();
            }
            if (getRuntime() - spinnerTimer >= 0.4) {
                spinnerTimer = 0;
            }

            if (this.gamepad2.left_trigger > 0.25) {
                launcher.setPosition(launchNectar);
            } else if (this.gamepad2.right_trigger > 0.25) {
                launcher.setPosition(launchPollen);
            }

            if (this.gamepad1.left_trigger > 0.25) {
                scale = 0.4;
            } else if (this.gamepad1.right_trigger > 0.25) {
                scale = 1;
            } else {
                scale = 0.8;
            }

            ManualDrive.driveOrHold(follower,-gamepad1.left_stick_y*scale, -gamepad1.left_stick_x*scale, -gamepad1.right_stick_x*scale);
            follower.update();

            spinner.setVelocity(spinnerPower[powerLevel]);

            telemetry.addData("Power Level", powerLevel);
            telemetry.addData("Spinner Velocity", spinner.getVelocity() / 28 * 60);
            telemetry.addData("Spinner Target Velocity", spinnerPower[powerLevel] / 28 * 60);
            telemetry.update();
        }
    }
}
