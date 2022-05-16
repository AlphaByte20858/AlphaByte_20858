package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Carrousel;
import org.firstinspires.ftc.teamcode.Hardware.motorTeleOp;
import org.firstinspires.ftc.teamcode.Hardware.servo;

@TeleOp(name = "TeleOp_Oficial_20858", group = "OpMode")
public class TeleOp_20858_oficial extends OpMode{
    motorTeleOp motor = new motorTeleOp();
    servo servoMotor = new servo();
    Carrousel carrousel = new Carrousel();

    @Override
    public void init() {
        motor.init();
    }

    @Override
    public void loop(){
        drive();
        arm();
        roda();

        telemetry.addData("motorPow", "motor Power: " + motor.mFE);
        telemetry.addData("motorPow", "motor Power: " + motor.mFD);
        telemetry.addData("motorPow", "motor Power: " + motor.mTE);
        telemetry.addData("motorPow", "motor Power: " + motor.mTD);
    }

    private void roda() {carrousel.climbOpMode();}

    private void drive() {
        double driveLeft;
        double driveRight;
        double driveForward;
        double driveBackward;

        driveLeft = -gamepad1.left_stick_x;
        driveRight = gamepad1.left_stick_x;
        driveForward = gamepad1.right_trigger;
        driveBackward = -gamepad1.left_trigger;

        //FORWARD
        motor.mFD.setPower(driveForward);
        motor.mFE.setPower(driveForward);
        motor.mTE.setPower(driveForward);
        motor.mTD.setPower(driveForward);

        //BACKWARD
        motor.mFD.setPower(driveBackward);
        motor.mFE.setPower(driveBackward);
        motor.mTE.setPower(driveBackward);
        motor.mTD.setPower(driveBackward);

        //LEFTWARD - FRONT
        motor.mFE.setPower(driveLeft);
        motor.mTE.setPower(driveLeft);

        //RIGHTWARD - FRONT
        motor.mFD.setPower(driveRight);
        motor.mTD.setPower(driveRight);
    }

    private void arm() { servoMotor.climbOpMode();}

}