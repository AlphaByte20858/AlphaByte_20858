package Autonomo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


public class Trajetory_ItemPersonalizado_blue extends LinearOpMode {

    @Override
    public void runOpMode() {

        Autonomous_20858 autonomous = new Autonomous_20858();
        Sensor_Color sensor = new Sensor_Color();
        Servo_Autonomous serving = new Servo_Autonomous();

        autonomous.moveRobot(100,100,100,100,1,1,1,1);
        serving.servoArm.setPosition(0.5);
        serving.servoElbow.setPosition(1);
        serving.servoHand.setPosition(1);

        sleep(1000);

        autonomous.moveRobot(-70,0,-70,0,-1,-1,-1,-1);
        autonomous.moveRobot(200,200,200,200,1,1,1,1);
        autonomous.moveRobot(-100,-50,-100,-50,-0.8,-0.8,-0.8,-0.8);

        if(sensor.verification)
        {
            autonomous.moveRobot(50,50,50,50,0.5,0.5,0.5,0.5);

        }



    }
}
