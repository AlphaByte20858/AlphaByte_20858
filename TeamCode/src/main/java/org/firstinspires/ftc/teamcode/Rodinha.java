package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Rodinha", group = "Auto")

public class Rodinha extends LinearOpMode {

    public boolean active = false;

    public DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {

        if(gamepad1.a && !active)
        {
            active = true;
            motor.setPower(0.5);
            sleep(500);
        }
        if(gamepad1.a && active)
        {
            motor.setPower(0);
        }
    }
}
