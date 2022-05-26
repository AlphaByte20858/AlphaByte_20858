package Autonomo;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Sensor: Color", group = "Sensor")
@Disabled

public class Sensor_Color extends LinearOpMode{

    NormalizedColorSensor sensor_color;

    View relativeLayout;

    final float[] hsvValues = new float[3];
    float ilu;
    boolean verification = false;

    @Override
    public void runOpMode(){

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        sensor_color = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        if (sensor_color instanceof SwitchableLight) {
            ((SwitchableLight)sensor_color).enableLight(true);
        }

        waitForStart();

        while (opModeIsActive()) {

            ilu += 0.05;

            sensor_color.setGain(ilu);

            NormalizedRGBA colors = sensor_color.getNormalizedColors();

            Color.colorToHSV(colors.toColor(), hsvValues);

            hsvValues[0] = 120;
            hsvValues[1] = 180;
            hsvValues[2] = 300;

            telemetry.addLine()
                    .addData("Red", "%.0f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.0f", colors.blue);

            if (sensor_color instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) sensor_color).getDistance(DistanceUnit.CM));
            }

            if(hsvValues[0] == 120 && hsvValues[1] == 180 && hsvValues[2] == 300)
            {
                verification = true;
            }

        }

    }
}
