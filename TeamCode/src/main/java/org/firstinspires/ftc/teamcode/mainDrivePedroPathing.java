package org.firstinspires.ftc.teamcode;

import static java.lang.Math.atan2;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name="mainDrive", group="mainDrive")
public class mainDrivePedroPathing extends LinearOpMode {
    private DcMotor intake;
    private Follower follower;
    private DcMotorEx spinner;
    private ServoImplEx launcherBackplate, flower0, flower1;
    private CRServoImplEx transfer0, transfer1, transfer2, launcher0, launcher1;

    @Override
    public void runOpMode(){
        spinner = hardwareMap.get(DcMotorEx.class, "spinner");
        intake = hardwareMap.get(DcMotor.class, "intake");
        spinner.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcherBackplate = hardwareMap.get(ServoImplEx.class, "launcher");
        flower0 = hardwareMap.get(ServoImplEx.class, "flower0");
        flower1 = hardwareMap.get(ServoImplEx.class, "flower1");
        launcherBackplate.setPwmRange(new PwmControl.PwmRange(500, 2500));
        flower0.setPwmRange(new PwmControl.PwmRange(500,2500));
        flower1.setPwmRange(new PwmControl.PwmRange(500,2500));
        
        transfer0 = hardwareMap.get(CRServoImplEx.class,"transfer0");
        transfer1 = hardwareMap.get(CRServoImplEx.class,"transfer1");
        transfer2 = hardwareMap.get(CRServoImplEx.class,"transfer2");
        launcher0 = hardwareMap.get(CRServoImplEx.class,"launcher0");
        launcher1 = hardwareMap.get(CRServoImplEx.class,"launcher1");
        transfer0.setPwmRange(new PwmControl.PwmRange(500, 2500));
        transfer1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        transfer2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        launcher0.setPwmRange(new PwmControl.PwmRange(500, 2500));
        launcher1.setPwmRange(new PwmControl.PwmRange(500, 2500));

        double spinnerP = 18, spinnerI = 0, spinnerD = 0, spinnerF = 14;
        PIDFCoefficients pidfSpinner = new PIDFCoefficients(spinnerP, spinnerI, spinnerD, spinnerF);
        spinner.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfSpinner);

        double scale; // for the movement of the robot
        follower = Constants.create(hardwareMap);
        final PoseFactory p = PoseFactory.degrees();

        int powerLevel = 0;
        double spinnerTimer = 0;

        double[] spinnerPower = {0, 2900, 3000, 3300, 3600, 3900, 4200};

        double launchPollen = 0.22, launchNectar = 0.43, flowerPickup0Up=0, flowerPickUp1Up=0, flowerPickup0Down=0, flowerPickUp1Down=0;

        follower.setPose(p.of(58, 8, 90));

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        launcherBackplate.setPosition(launchNectar);

        while(opModeIsActive()){
            if(this.gamepad2.left_trigger_pressed){
                //forward direction
                transfer0.setPower(1);
                transfer1.setPower(1);
                transfer2.setPower(1);
            } else if (this.gamepad2.right_trigger_pressed) {
                //reverse direction
                transfer0.setPower(-1);
                transfer1.setPower(-1);
                transfer2.setPower(-1);
            } else {
                transfer0.setPower(0);
                transfer1.setPower(0);
                transfer2.setPower(0);
            }

            if (this.gamepad2.a){
                //must be opposite for them to push in same direction
                launcher0.setPower(1);
                launcher1.setPower(-1);
            } else {
                launcher0.setPower(0);
                launcher1.setPower(0);
            }

            if (this.gamepad2.b){
                //code for up state
                flower0.setPosition(flowerPickup0Up);
                flower1.setPosition(flowerPickUp1Up);
            } else if (this.gamepad2.x) {
                //code for down state
                flower0.setPosition(flowerPickup0Down);
                flower1.setPosition(flowerPickUp1Down);
            }

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
                launcherBackplate.setPosition(launchNectar);
            } else if (this.gamepad2.right_trigger > 0.25) {
                launcherBackplate.setPosition(launchPollen);
            }

            if (this.gamepad1.left_trigger > 0.25) {
                scale = 0.4;
            } else if (this.gamepad1.right_trigger > 0.25) {
                scale = 1;
            } else {
                scale = 0.8;
            }


            //Reset positioning
            if(this.gamepad1.a){
                Pose cornerPose = new Pose(10.5, 10.5, Math.toRadians(90));
                follower.setPose(cornerPose);
            }
            //rotate to goal, one side, 58,58 is the goal but will need to change
            if(this.gamepad1.b){
                double angle = atan2(follower.pose().y()-58,follower.pose().x()-58);
                DrivePowers powers = ManualDrive.fieldCentric(
                        -gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad1.right_stick_x,
                        angle
                );
                ManualDrive.driveOrHold(follower, powers);
                follower.update();
            }
            //rotate to goal, other side
            if(this.gamepad1.x){

            }

            ManualDrive.driveOrHold(follower,-gamepad1.left_stick_y*scale, -gamepad1.left_stick_x*scale, -gamepad1.right_stick_x*scale);
            follower.update();

            spinner.setVelocity(spinnerPower[powerLevel]/60.0*28);

            telemetry.addData("Power Level", powerLevel);
            telemetry.addData("Spinner Velocity", spinner.getVelocity() / 28 * 60);
            telemetry.addData("Spinner Target Velocity", spinnerPower[powerLevel]);
            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("rotation", Math.toDegrees(follower.pose().heading()));
            telemetry.update();
        }
    }
}
