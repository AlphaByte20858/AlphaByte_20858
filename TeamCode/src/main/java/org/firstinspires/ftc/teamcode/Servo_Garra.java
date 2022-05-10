package Autonomo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name = "ServoGarra_Autonomous", group = "Auto")
@Disabled

public class Servo_Garra extends LinearOpMode{
    static final double INCREMENT = 0.2;
    static final int CYCLE_MS = 50;
    static final double MAX_POS = 1.0;
    static final double MIN_POS = 0.0;

    Servo servo2;
    double position = MIN_POS;
    boolean open = true; //ato de abrir
    boolean buttonX = gamepad1.x;
    double POS_Y = MIN_POS;

   public void climbOpMode()
   {
      servo2 = hardwareMap.get(Servo.class, "hand");
   }

    @Override
    public void runOpMode()
    {
          climbOpMode();
    }
}
