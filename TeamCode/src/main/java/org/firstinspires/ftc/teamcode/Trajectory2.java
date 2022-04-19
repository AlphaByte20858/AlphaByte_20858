package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMROpticalDistance;


@Autonomous(name = "Trajetory_armazem,autonomous", group = "Auto")
@Disabled

public class Trajectory2 extends LinearOpMode {
    @Override
    public void runOpMode() {
        Move_Encoder_Autonomous move = new Move_Encoder_Autonomous();
        MoveServo servo = new MoveServo();
        SensorMROpticalDistance sensorDistance = new SensorMROpticalDistance();

        move.moveBot(-1, -1, 1, 1, 1);
        move.moveBot(1000, 1000, 1000, 1000, 1);
        servo.climbOpMode();

    }

}
