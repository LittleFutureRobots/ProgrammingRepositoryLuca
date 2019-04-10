package org.firstinspires.ftc.teamcode.ProgrammingFTC2019LFR;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Sensor: REV2mDistance", group = "Sensor")
public class MovingStraightwithRangeSensor extends LinearOpMode {
    DcMotor frontleft, frontright, backleft, backright;
    private DistanceSensor sensorRange;

    @Override
    public void runOpMode() {

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor1");
        frontleft = hardwareMap.dcMotor.get("frrontleft");
        frontright = hardwareMap.dcMotor.get("frontright");
        backleft = hardwareMap.dcMotor.get("backleft");
        backright = hardwareMap.dcMotor.get("backright");

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorRange;

        waitForStart();
        MoveForward();

    }

    public void MoveForward() {
        if (sensorRange.getDistance(DistanceUnit.CM) > 6) {
            frontleft.setPower(0.2);
            frontright.setPower(0.2);
            backleft.setPower(0.2);
            backright.setPower(0.2);
        } else {
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);

        }
    }

}
