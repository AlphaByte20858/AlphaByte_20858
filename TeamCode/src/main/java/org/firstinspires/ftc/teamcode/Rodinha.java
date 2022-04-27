package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Rodinha", group = "Auto")

public class Rodinha extends LinearOpMode {

    public boolean active = false;

    public DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {

        Move_Encoder_Autonomous move = new Move_Encoder_Autonomous();

        if(gamepad1.a && !active)
        {
            active = true;
            motor.setPower(0.5);
            sleep(500);
            active = false;
            motor.setPower(0);

        }
    }
}
