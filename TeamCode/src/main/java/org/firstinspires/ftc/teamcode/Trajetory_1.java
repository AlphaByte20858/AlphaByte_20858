package Autonomo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Trajetory_1", group = "Auto")
@Disabled

public class Trajetory_1 extends LinearOpMode{

    @Override
    public void runOpMode() {
        Autonomous_20858 motor = new Autonomous_20858();
        Servo_Autonomous servo = new Servo_Autonomous();

        motor.moveRobot(-1,-1,1,1,-1,-1,1,1);
        motor.moveRobot(1,1,1,1,1,1,1,1);
        servo.opModeIsActive();

        sleep(1000);

        motor.moveRobot(1,1,1,1,-1,-1,-1,-1);
        motor.moveRobot(1,1,-1,-1,1,1,-1,-1);
        motor.moveRobot(0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5);

        servo.servoArm.setPosition(-1);
        servo.servoHand.setPosition(1);

        sleep(500);

        motor.moveRobot(-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5);



    }
}
