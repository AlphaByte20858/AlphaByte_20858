package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@TeleOp(name = "Concept: MoveServo" , group = "Concept")
@Disabled



public abstract class MoveServo extends LinearOpMode {

    static final double INCREMENT = 0.2;     // amount to slew servo each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position


    Servo servo;
    double position = MIN_POS;
    boolean Up = true;  // Ato de subir
    boolean buttonY = gamepad1.y;
    double POS_Y = MIN_POS;
    double intermed = (MAX_POS - MIN_POS) / 2;
    float buttonL1 = gamepad1.right_trigger;



    public void climbOpMode() {
        servo = hardwareMap.get(Servo.class, "right_hand");
        servo.setPosition(Servo.MIN_POSITION);

        telemetry.addData(">", "Press Start to scan Servo.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if(buttonY)
            {
                position += INCREMENT;
                POS_Y += INCREMENT;

                if(position>= MAX_POS)
                {
                    POS_Y = MAX_POS;
                    position = MAX_POS;
                    Up = !Up;

                }
            }

            if()

            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new position and pause;
            servo.setPosition(position);
            servo.setPosition(POS_Y);
            sleep(CYCLE_MS);
            idle();

            telemetry.addData(">", "Done");
            telemetry.update();

        }

        }
    }




