package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp_Oficial_20858", group = "OpMode")
public class TeleOp_20858 extends OpMode{
    public DcMotor mFE, mTE, mFD, mTD;
    public DcMotor coreHex1, coreHex2;

    boolean active = true;

    Servo servoArm;
    Servo servoFist;
    Servo servoBase;
    Servo servoHand;

    double basePos = 0.0;
    double armPos = 0.0;
    double fistPos = 0.0;
    double handPos = 0.0;

    final double speed = 0.01;

    @Override
    public void init() {
        mFE = hardwareMap.get(DcMotor.class, "FL");
        mFD = hardwareMap.get(DcMotor.class, "FR");
        mTE= hardwareMap.get(DcMotor.class, "BL");
        mTD = hardwareMap.get(DcMotor.class, "BR");

        coreHex1 = hardwareMap.get(DcMotor.class, "coreHex");
        coreHex2 = hardwareMap.get(DcMotor.class, "coreHex2");
        coreHex1.setDirection(DcMotor.Direction.FORWARD);
        coreHex2.setDirection(DcMotor.Direction.FORWARD);


        mFE.setDirection(DcMotor.Direction.REVERSE);
        mFD.setDirection(DcMotor.Direction.FORWARD);
        mTE.setDirection(DcMotor.Direction.REVERSE);
        mTD.setDirection(DcMotor.Direction.FORWARD);

        mTE.getCurrentPosition();
        mTD.getCurrentPosition();
        mFE.getCurrentPosition();
        mFD.getCurrentPosition();


        servoBase = hardwareMap.get(Servo.class, "base");
        servoArm = hardwareMap.get(Servo.class, "arm");
        servoHand = hardwareMap.get(Servo.class, "hand");
        servoFist = hardwareMap.get(Servo.class, "fist");

        servoArm.setPosition(armPos);
        servoHand.setPosition(handPos);
        servoBase.setPosition(basePos);
        servoFist.setPosition(fistPos);

    }
    public void motorPower(float powLF, float powLB, float powRF, float powRB){
        mFE.setPower(powLF);
        mTE.setPower(powLB);
        mFD.setPower(powRF);
        mTD.setPower(powRB);
    }

    @Override
    public void loop(){
        drive();
        arm();
        roda();

        telemetry.addData("motorPow", "motor Power: " + mFE);
        telemetry.addData("motorPow", "motor Power: " + mFD);
        telemetry.addData("motorPow", "motor Power: " + mTE);
        telemetry.addData("motorPow", "motor Power: " + mTD);
    }

    private void roda() {
        if(gamepad1.y && !active){
            coreHex1.setPower(0.7);
            coreHex2.setPower(0.7);
        }
        else if(gamepad1.y && active){
            coreHex1.setPower(0.0);
            coreHex2.setPower(0.0);
        }
    }

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
        mFD.setPower(driveForward);
        mFE.setPower(driveForward);
        mTE.setPower(driveForward);
        mTD.setPower(driveForward);

        //BACKWARD
        mFD.setPower(driveBackward);
        mFE.setPower(driveBackward);
        mTE.setPower(driveBackward);
        mTD.setPower(driveBackward);

        //LEFTWARD - FRONT
        mFE.setPower(driveLeft);
        mTE.setPower(driveLeft);

        //RIGHTWARD - FRONT
        mFD.setPower(driveRight);
        mTD.setPower(driveRight);
    }

    private void arm() {
        if(gamepad2.y && active){
            basePos += speed;
            armPos += speed;
            fistPos += speed;
            handPos += speed;
        }
        else if(gamepad2.y){
            basePos -= speed;
            armPos -= speed;
            fistPos -= speed;
            handPos -= speed;
        }
    }

}
