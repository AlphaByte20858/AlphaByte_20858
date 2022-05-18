package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@TeleOp(name = "Concept: MoveServo" , group = "Concept")
@Disabled
public class servo extends LinearOpMode{

    static final double INCREMENT = 0.2;     // amount to slew servo each CYCLE_MS cycle
    static final int CYCLE_MS = 500;     // period of each cycle
    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position


    Servo servoArm;
    Servo servoHand;
    double position = MIN_POS;



    public void climbOpMode() {
        servoArm = hardwareMap.get(Servo.class, "arm");
        servoHand = hardwareMap.get(Servo.class, "hand");
        servoArm.setPosition(Servo.MAX_POSITION);

        telemetry.addData(">", "Press Start to scan Servo.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
              //ARM
          if(gamepad2.y && !up){
              servoArm.setPosition(MAX_POS);
          }
          if(gamepad2.y && up){
              servoArm.setPosition(MIN_POS);
          }
           //HAND
          if(gamepad2.x && !open){
              servoHand.setPosition(MAX_POS);
          }
          if(gamepad2.x && open){
              servoHand.setPosition(MIN_POS);
          }


            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new position and pause;
            servoArm.setPosition(position);
            sleep(CYCLE_MS);
            idle();

            telemetry.addData(">", "Done");
            telemetry.update();

        }

        }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}




