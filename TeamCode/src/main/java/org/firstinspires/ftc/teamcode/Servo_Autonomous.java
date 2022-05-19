package Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Servo_Auto", group = "Auto")
@Disabled

public class Servo_Autonomous extends LinearOpMode{

    static final int CYCLE_MS = 500;     // period of each cycle
    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position


    Servo servoArm;
    Servo servoHand;
    Servo servoElbow;



    @Override
    public void runOpMode() {

        servoArm = hardwareMap.get(Servo.class, "arm");
        servoHand = hardwareMap.get(Servo.class, "hand");
        servoElbow = hardwareMap.get(Servo.class, "elbow");
        servoArm.setPosition(Servo.MAX_POSITION);
        servoElbow.setPosition(Servo.MIN_POSITION);
        servoHand.setPosition((Servo.MIN_POSITION));

        Sensor_Toque galileo = new Sensor_Toque();




        telemetry.addData(">", "Press Start to scan Servo.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if(!galileo.toque.getState()){
                servoArm.setPosition(MIN_POS);
                sleep(500);
                servoHand.setPosition(MAX_POS);
                sleep(500);
                servoElbow.setPosition(MAX_POS);
            }



    }
  }
}
