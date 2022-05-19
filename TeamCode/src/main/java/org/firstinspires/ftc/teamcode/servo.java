package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@TeleOp(name = "Concept: MoveServo" , group = "Concept")
@Disabled
public class servo extends LinearOpMode{

    Servo servoArm;
    Servo servoFist;
    Servo servoBase;
    Servo servoHand;

    double basePos = 0.0;
    double armPos = 0.0;
    double fistPos = 0.0;
    double handPos = 0.0;

    final double speed = 0.01;

    public void climbOpMode() {
        servoBase = hardwareMap.get(Servo.class, "base");
        servoArm = hardwareMap.get(Servo.class, "arm");
        servoHand = hardwareMap.get(Servo.class, "hand");
        servoFist = hardwareMap.get(Servo.class, "fist");

        servoArm.setPosition(armPos);
        servoHand.setPosition(handPos);
        servoBase.setPosition(basePos);
        servoFist.setPosition(fistPos);

        while (opModeIsActive()) {
            if(gamepad2.y){
               basePos += speed;
               armPos += speed;
               fistPos += speed;
               handPos += speed;
            }
            else if(gamepad2.x){
                basePos -= speed;
                armPos -= speed;
                fistPos -= speed;
                handPos -= speed;
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}







