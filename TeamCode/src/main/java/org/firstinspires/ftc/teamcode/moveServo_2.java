package TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Encorder_Garra2", group = "Servo")
@Disabled
public class moveServo_2 extends LinearOpMode {
    static final double INCREMENT = 0.2;
    static final int CYCLE_MS = 50;
    static final double MAX_POS = 1.0;
    static final double MIN_POS = 0.0;

    Servo servo2;
    double position = MIN_POS;
    boolean open = true; //ato de abrir
    boolean buttonX = gamepad1.x;
    double POS_Y = MIN_POS;


    public void climbOpMode(){
        servo2 = hardwareMap.get(Servo.class, "hand");
        servo2.setPosition(Servo.MIN_POSITION);

        while(opModeIsActive()){
            if(buttonX){
                position += INCREMENT;
                POS_Y += INCREMENT;
            }
            if(position>= MAX_POS){
                POS_Y = MAX_POS;
                position = MAX_POS;
                open = !open;
            }
        }

        servo2.setPosition(position);
        servo2.setPosition(POS_Y);
        sleep(CYCLE_MS);
        idle();
    }


    @Override
    public void runOpMode() throws InterruptedException {

    }
}
