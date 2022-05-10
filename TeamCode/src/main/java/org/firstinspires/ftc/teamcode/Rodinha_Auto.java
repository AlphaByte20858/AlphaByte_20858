package Autonomo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Rodinha_Auto", group = "Auto")

public class Rodinha_Auto extends LinearOpMode{

    public DcMotor motor_auto;

    public boolean active_auto = false;

    public float time = 20;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData(">", "Press Start.");
        telemetry.update();

        waitForStart();
        while(opModeIsActive())
        {
            time -= 1;
            active_auto = true;
            motor_auto.setPower(0.5);

            if(time >= 0)
            {
                break;
            }

        }


    }
}
