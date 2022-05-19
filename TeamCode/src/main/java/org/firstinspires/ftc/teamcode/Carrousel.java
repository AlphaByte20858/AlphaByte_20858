package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Carrousel", group = "Auto")
@Disabled
public class Carrousel extends LinearOpMode {
    public DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, "coreHex");
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.getCurrentPosition();
    }

    public void climbOpMode(){
        double runMotor;

        runMotor = gamepad2.right_trigger;
        motor.setPower(runMotor);
    }
}
