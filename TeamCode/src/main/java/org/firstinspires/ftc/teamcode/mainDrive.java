package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="mainDrive", group="mainDrive")
public class mainDrive extends LinearOpMode {
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