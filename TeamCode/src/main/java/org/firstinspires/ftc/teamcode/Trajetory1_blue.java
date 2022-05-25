package Autonomo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Trajetory_1", group = "Auto")
@Disabled

public class Trajetory1_blue extends LinearOpMode{

    @Override
    public void runOpMode() {
        Autonomous_20858 motor = new Autonomous_20858();
        Servo_Autonomous servo = new Servo_Autonomous();

        waitForStart();

        while(opModeIsActive()) {

            motor.moveRobot(-100, -100, 100, 100, -1, -1, 1, 1);
            motor.moveRobot(100, 100, 100, 100, 1, 1, 1, 1);

            sleep(500);

            servo.opModeIsActive();

            sleep(500);

            servo.servoArm.setPosition(0.5);

            motor.moveRobot(100, 100, 100, 100, -1, -1, -1, -1);
            motor.moveRobot(100, 100, -100, -100, 1, 1, -1, -1);
            motor.moveRobot(50, 50, 50, 50, 0.8, 0.8, 0.8, 0.8);

            servo.servoHand.setPosition(1);

            sleep(500);

            motor.moveRobot(-50, -50, -50, -50, -0.5, -0.5, -0.5, -0.5);
        }

    }
}
