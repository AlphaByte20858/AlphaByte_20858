package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Sensor_Cor extends LinearOpMode {
    NormalizedColorSensor cor;
    View layout;

    @Override
    public void runOpMode() throws InterruptedException {
      int layoutId = hardwareMap.appContext.getResources().getIdentifier("layout", "id", hardwareMap.appContext.getPackageName());
      layout = ((Activity) hardwareMap.appContext).findViewById(layoutId);

      try{
          runSample();
      } finally {
          layout.post(new Runnable() {
              @Override
              public void run() {
                  layout.setBackgroundColor(Color.WHITE);
              }
          });
      }
    }

    private void runSample() {
        float gain = 2;
        final float[] hasValues = new float [3];
        boolean buttonPrevX = false;
        boolean buttonCurX = false;

        cor = hardwareMap.get(NormalizedColorSensor.class, "Sensor_Cor");

        if (cor instanceof SwitchableLight){
            ((SwitchableLight)cor).enableLight(true);
        }

        while(opModeIsActive()){
            telemetry.addLine("Presisone A para aumentar Ganho e B para reduzir Ganho.\n");
            telemetry.addLine("Altos valores de ganho significa que os sensores vão mostar grandes númeors para RGB");

            if(gamepad2.a){
                gain += 0.005;
            } else if (gamepad2.b && gain > 1){
                gain -= 0.005;
            }
            telemetry.addData("Gain", gain);
            cor.setGain(gain);
            buttonCurX = gamepad2.x;

            if(buttonCurX != buttonPrevX){
                if(buttonCurX){
                    if(cor instanceof SwitchableLight){
                        SwitchableLight light = (SwitchableLight)cor;
                        light.enableLight(!light.isLightOn());
                    }
                }
            }
            buttonPrevX = buttonCurX;
            NormalizedRGBA colors = cor.getNormalizedColors();

            Color.colorToHSV(colors.toColor(), hasValues);

            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("green", "%.3f", colors.green)
                    .addData("blue", "%.3f", colors.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hasValues[0])
                    .addData("Saturation", "%.3f", hasValues[1])
                    .addData("Value", "%.3f", hasValues[2]);
            telemetry.addData("Alpha", "%.3f", colors.alpha);

            if(cor instanceof DistanceSensor){
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) cor).getDistance(DistanceUnit.CM));
            }
            telemetry.update();

            layout.post(new Runnable() {
                @Override
                public void run() {
                    layout.setBackgroundColor(Color.HSVToColor(hasValues));
                }
            });
        }
    }
}
