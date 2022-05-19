package Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Tajetory_Auto", group = "Auto")
@Disabled

public class Trajetory_Duck extends LinearOpMode {

    Autonomous_20858 motor = new Autonomous_20858();
    Roulette_Auto roulette = new Roulette_Auto();

    @Override
    public void runOpMode(){

        motor.moveRobot(1,1,-1,-1,1,1,-1,-1);
        motor.moveRobot(1,1,1,1,1,1,1,1);
        roulette.runOpMode();
        sleep(2000);
        motor.moveRobot(-1,-1,-1,-1,-1,-1,-1,-1);

    }
}
