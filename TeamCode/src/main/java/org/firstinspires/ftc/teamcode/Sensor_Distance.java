package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Sensor: REV", group = "sensor")
public class Sensor_Distance extends LinearOpMode {
    private DistanceSensor distancia_Sensor;


    @Override
    public void runOpMode() throws InterruptedException {
        distancia_Sensor = hardwareMap.get(DistanceSensor.class, "sensor_Distancia");

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)distancia_Sensor;

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("deviceName", distancia_Sensor.getDeviceName());
            telemetry.addData("distance", String.format("%.01f m", distancia_Sensor.getDistance(DistanceUnit.METER)));


            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            telemetry.addData("time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

            telemetry.update();
        }
    }
}
