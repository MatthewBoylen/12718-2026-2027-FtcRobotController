package org.firstinspires.ftc.teamcode;
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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp(name="Drive", group="Drive")
public class Drive extends LinearOpMode {
    private DcMotor frontLeft, backLeft, frontRight, backRight;
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
        waitForStart();

        while (opModeIsActive()) {

            direction = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x);
            turn = -gamepad1.right_stick_x;
            speed = Math.sqrt((gamepad1.left_stick_y*gamepad1.left_stick_y)+(gamepad1.left_stick_x*gamepad1.left_stick_x));
            frontLeft.setPower(Math.sin(direction+(0.25*(Math.PI)))*speed+turn);
            frontRight.setPower(Math.sin(direction-(0.25*(Math.PI)))*speed-turn);
            backLeft.setPower(Math.sin(direction-(0.25*(Math.PI)))*speed+turn);
            backRight.setPower(Math.sin(direction+(0.25*(Math.PI)))*speed-turn);
        }
    }
}