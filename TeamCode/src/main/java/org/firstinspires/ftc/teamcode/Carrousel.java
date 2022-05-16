package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Carrousel", group = "Auto")
@Disabled
public class Carrousel extends LinearOpMode {

    public boolean active = false;
    public DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {
    }

    public void climbOpMode(){
        if(gamepad2.y && !active)
        {
            active = true;
            motor.setPower(0.5);
            sleep(500);
        }
        if(gamepad2.a && active)
        {
            motor.setPower(0);
        }
    }
}
