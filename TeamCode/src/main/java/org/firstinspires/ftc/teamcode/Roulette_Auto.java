package Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Roulette", group = "Auto")
@Disabled

public class Roulette_Auto extends LinearOpMode {

    public DcMotor dcMotor;
    public boolean active = true;

    @Override
    public void runOpMode() {

        while(active) {
            dcMotor.setPower(1);
            sleep(2000);

            for(double time = 0; time <= 20; time++)
            {
                if(time >= 20)
                {
                    break;
                }
            }
        }
    }
}
