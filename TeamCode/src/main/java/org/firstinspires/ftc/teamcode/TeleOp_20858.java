package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp_Oficial_20858", group = "OpMode")
public class TeleOp_20858 extends OpMode {
    public DcMotor mFE, mTE, mFD, mTD;
    public DcMotor coreHex1, coreHex2;

    Servo servoArm;
    Servo servoBase;
    Servo servoHand;

    @Override
    public void init() {
        mFE = hardwareMap.get(DcMotor.class, "FL");
        mFD = hardwareMap.get(DcMotor.class, "FR");
        mTE = hardwareMap.get(DcMotor.class, "BL");
        mTD = hardwareMap.get(DcMotor.class, "BR");

        coreHex1 = hardwareMap.get(DcMotor.class, "coreHex");
        coreHex2 = hardwareMap.get(DcMotor.class, "coreHex2");
        coreHex1.setDirection(DcMotor.Direction.FORWARD);
        coreHex2.setDirection(DcMotor.Direction.FORWARD);


        mFE.setDirection(DcMotor.Direction.FORWARD);
        mFD.setDirection(DcMotor.Direction.REVERSE);
        mTE.setDirection(DcMotor.Direction.FORWARD);
        mTD.setDirection(DcMotor.Direction.REVERSE);

        mTE.getCurrentPosition();
        mTD.getCurrentPosition();
        mFE.getCurrentPosition();
        mFD.getCurrentPosition();


        servoBase = hardwareMap.get(Servo.class, "base");
        servoArm = hardwareMap.get(Servo.class, "arm");
        servoHand = hardwareMap.get(Servo.class, "hand");

    }

    public void motorPower(float powLF, float powLB, float powRF, float powRB) {
        mFE.setPower(powLF);
        mTE.setPower(powLB);
        mFD.setPower(powRF);
        mTD.setPower(powRB);
    }

    @Override
    public void loop() {
        drive();
        arm();
        roda();

        telemetry.addData("motorPow", "motor Power: " + mFE);
        telemetry.addData("motorPow", "motor Power: " + mFD);
        telemetry.addData("motorPow", "motor Power: " + mTE);
        telemetry.addData("motorPow", "motor Power: " + mTD);
    }

    private void roda() {
        if (gamepad1.right_bumper) {
            coreHex1.setPower(0.7);
            coreHex2.setPower(0.7);
        }
        if (gamepad1.left_bumper) {
            coreHex1.setPower(-0.7);
            coreHex2.setPower(-0.7);
        } else if (gamepad1.b) {
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
      boolean front;
      boolean frontArm;
      boolean backArm;
      boolean back;
      boolean up;
      boolean down;


      front = gamepad2.right_bumper;
      frontArm = gamepad2.x;
      backArm = gamepad2.y;
      back = gamepad2.left_bumper;
      up = gamepad2.dpad_up;
      down = gamepad2.dpad_down;

      if(front){
          servoBase.setPosition(1);
      }
      if(back){
          servoBase.setPosition(0);
      }
      if(frontArm){
          servoArm.setPosition(1);
      }
      if(backArm){
          servoArm.setPosition(0);
      }
      if(up){
          servoHand.setPosition(1);
      }
      if(down){
          servoHand.setPosition(0);
      }
    }
}
