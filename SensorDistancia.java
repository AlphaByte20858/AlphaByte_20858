package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.opmode.LinearOpMode;
import com.qualcomm.robotcore.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Sensor_de_Distancia", group = "Sensor")
public class SensorDistancia extends LinearOpMode{

    ModernRoboticsI2cRangeSensor distanciaSensor;

    @Override
    public void runOpMode(){
        distanciaSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_distancia");

        while (opModeIsActive()){
            telemetry.addData("ultrasonica raw", distanciaSensor.rawUltrasonic());
            telemetry.addData("optical raw", distanciaSensor.rawOptical());
            telemetry.addData("cm optical", "%.2f cm", distanciaSensor.cmOptical());
            telemetry.addData("cm", "%.2f", distanciaSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}