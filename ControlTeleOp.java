package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "ControlTeleOp", group = "Controls FTC 2019")
public class ControlTeleOP extends LinearOpMode {
    private DcMotor frontleft, frontright, backleft, backright, motorarm0, motorarm1, motorarm2, motorarm3;
    private Servo servo0, servo1;
    private CRServo servo2;

    @Override
    public void runOpMode() {
        frontleft = hardwareMap.dcMotor.get("frontleft");
        frontright = hardwareMap.dcMotor.get("frontright");
        backleft = hardwareMap.dcMotor.get("backleft");
        backright = hardwareMap.dcMotor.get("backright");
        motorarm0 = hardwareMap.dcMotor.get("motorarm0");
        motorarm1 = hardwareMap.dcMotor.get("motorarm1");
        motorarm2 = hardwareMap.dcMotor.get("motorarm2");
        motorarm3 = hardwareMap.dcMotor.get("motorarm3");
        servo0 = hardwareMap.servo.get("servo0");
        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.crservo.get("servo2");

        motorarm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorarm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                motorarm0.setPower(1.0);
            } else if (gamepad1.y) {
                motorarm0.setPower(-1.0);
            } else {
                motorarm0.setPower(0);
            }
            if (gamepad1.right_bumper) {
                motorarm3.setPower(-1);
            } else if (gamepad1.left_bumper) {
                motorarm3.setPower(1);
            } else {
                motorarm3.setPower(0);
            }
            if (gamepad1.dpad_up) {
                servo1.setPosition(1);
            } else if (gamepad1.dpad_down) {
                servo1.setPosition(-1);
            }
            if (gamepad1.dpad_left) {
                servo2.setPower(1);
            } else if (gamepad1.dpad_right) {
                servo2.setPower(-1);
            } else if (gamepad1.b) {
                servo2.setPower(0);
            }
            if (gamepad1.back) {
                servo0.setPosition(0.5);
            }
            if (gamepad1.x) {
                servo0.setPosition(0);
            }
            frontleft.setPower(Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x, -1.0, 1.0));
            frontright.setPower(Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x, -1.0, 1.0));
            backleft.setPower(Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x, -1.0, 1.0));
            backright.setPower(Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x, -1.0, 1.0));
            motorarm1.setPower(Range.clip(-gamepad1.right_stick_y + gamepad1.right_stick_x, -0.3, 0.3));
            motorarm2.setPower(Range.clip(gamepad1.right_stick_y + gamepad1.right_stick_x, -0.7, 0.7));
        }

    }

}
