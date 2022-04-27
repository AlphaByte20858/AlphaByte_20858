package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name ="Trajetory_Duck", group = "Auto")
@Disabled


public class Trajetory_Duck extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        Move_Encoder_Autonomous move = new Move_Encoder_Autonomous();
        Rodinha roda = new Rodinha();

        move.moveBot(1,1,-1,-1,1);
        s


    }
}
