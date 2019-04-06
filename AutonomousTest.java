package org.firstinspires.ftc.teamcode.ProgrammingFTC2019LFR;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous
public class AutonomousTest extends LinearOpMode {
    AutonomousMakers Autonomie = new AutonomousMakers();


    @Override
    public void runOpMode() {
        Autonomie.initAll(hardwareMap);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addLine("Da ce ai facut bobita    ?");
            telemetry.update();

        }
        Autonomie.turnTo(0.3, -30);
        Autonomie.DriveForward(1);
        sleep(100);
        Autonomie.DriveForward(0);
        Autonomie.turnTo(0.3, 30);
        Autonomie.runObjectDetection(2, new AutonomousMakers.ObjectDetected() {
            @Override
            public void onLeft() {
                Autonomie.DriveForward(1);
                sleep(700);
                Autonomie.DriveForward(0);
                Autonomie.turnTo(0.5, -50);
                Autonomie.DriveForward(1);
                sleep(1200);
                Autonomie.TeamMarker(1);

            }

            @Override
            public void onCenter() {
                Autonomie.DriveForward(1);
                sleep(1500);
                Autonomie.DriveForward(0);
                Autonomie.TeamMarker(1);

            }

            @Override
            public void onRight() {
                Autonomie.DriveForward(1);
                sleep(700);
                Autonomie.DriveForward(0);
                Autonomie.turnTo(0.5, 50);
                Autonomie.DriveForward(1);
                sleep(1200);
                Autonomie.TeamMarker(1);

            }

        });
    }
}