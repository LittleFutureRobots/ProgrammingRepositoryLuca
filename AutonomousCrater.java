package org.firstinspires.ftc.teamcode.ProgrammingFTC2019LFR;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous
public class AutonomousCrater extends LinearOpMode {
    AutonomousConfigurationFile Autonomie = new AutonomousConfigurationFile();


    @Override
    public void runOpMode() {
        Autonomie.initAll(hardwareMap);
        waitForStart();
        Autonomie.MoveOnSensor(110);
        Autonomie.MoveForward(-1);
        sleep(1000);
        Autonomie.MoveForward(0);

        Autonomie.runObjectDetection(2, new AutonomousConfigurationFile.ObjectDetected() {
            @Override
            public void onLeft() {
                Autonomie.MoveOnSensor(80);
                Autonomie.MoveForward(-1);
                sleep(1000);
                Autonomie.MoveForward(0);
                Autonomie.turnTo(1,60);
                Autonomie.MoveOnSensor(60);

            }

            @Override
            public void onCenter() {



            }

            @Override
            public void onRight() {



            }

        });

    }
}




