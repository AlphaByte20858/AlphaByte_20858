package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp (name = "Sensor_toque", group = "Sensor")
@Disabled
public class Sensor_Toque extends LinearOpMode {

    DigitalChannel toque;

    @Override
    public void runOpMode() throws InterruptedException {
        toque = hardwareMap.get(DigitalChannel.class, "sensor_toque");
        toque.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();

        while(opModeIsActive()){
            if(toque.getState()){
                telemetry.addData("Sensor de Toque", "Não está pressionado");
            }
            else{
                telemetry.addData("Sensor de Toque", "Está pressionado");
            }
            telemetry.update();
        }
    }
}