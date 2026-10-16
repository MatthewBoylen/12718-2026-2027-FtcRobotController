package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="DrivetrainTest",group = "Test")
public class DrivetrainTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        DcMotor frontLeft=hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight=hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft=hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight=hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int index=0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.aWasPressed()){
                index++;
            }

            if (index==0){
                telemetry.addData("Testing", "frontLeft");
                frontLeft.setPower(1);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            } else if (index==1) {
                telemetry.addData("Testing", "frontRight");
                frontLeft.setPower(0);
                frontRight.setPower(1);
                backLeft.setPower(0);
                backRight.setPower(0);
            } else if (index==2) {
                telemetry.addData("Testing", "backLeft");
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(1);
                backRight.setPower(0);
            } else if (index==3) {
                telemetry.addData("Testing", "backRight");
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(1);
            }
        }
    }

}
