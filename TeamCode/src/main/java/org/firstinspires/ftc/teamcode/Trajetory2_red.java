package Autonomo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Trajetory_2", group = "Auto")
@Disabled

public class Trajetory2_red extends LinearOpMode {

    @Override
    public void runOpMode() {


        Autonomous_20858 auto = new Autonomous_20858();
        Servo_Autonomous servo = new Servo_Autonomous();
        Roulette_Auto carrousel = new Roulette_Auto();

        waitForStart();

        while (opModeIsActive()) {

            auto.moveRobot(50, 50, 50, 50, 0.5, 0.5, 0.5, 0.5);
            auto.moveRobot(30, 30, 0, 0, 0.3, 0, 0.3, 0);
            auto.moveRobot(20, 20, 20, 20, 0.2, 0.2, 0.2, 0.2);

            servo.servoArm.setPosition(0.5);
            servo.servoElbow.setPosition(0.4);
            servo.servoHand.setPosition(0.1);

            sleep(500);

            auto.moveRobot(-50, -50, -50, -50, -0.5, -0.5, -0.5, -0.5);
            auto.moveRobot(0, 0, -100, -100, 0, -1, 0, -1);
            auto.moveRobot(0, 0, -20, -20, 0, 0, -20, -20);
            auto.moveRobot(20, 20, 20, 20, 0.2, 0.2, 0.2, 0.2);

            carrousel.dcMotor.setPower(0.7);
            sleep(500);

            auto.moveRobot(0, 0, -20, -20, 0, 0, -0.5, -0.5);
            auto.moveRobot(-50, -50, -50, -50, -0.5, -0.5, -0.5, -0.5);

        }
    }

 }

