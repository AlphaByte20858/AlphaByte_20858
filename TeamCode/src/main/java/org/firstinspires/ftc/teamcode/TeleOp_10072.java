package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Basic_TeleOp", group = "TeleOp")
public class TeleOp_10072 extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    public DcMotor FL, BL, FR, BR;

    /*********************************************
     * FL = FRONT LEFT
     * BL = BACK LEFT
     * FR = FRONT RIGHT
     * BR = BACK RIGHT
     ********************************************/

    static final double INCREMENT = 0.2;     
    static final int CYCLE_MS = 50;     
    static final double MAX_POS = 1.0;     
    static final double MIN_POS = 0.0;


    Servo servo;
    double position = MIN_POS;
    boolean Up = true; //ato de subir 
    boolean buttonY = gamepad2.y;
    double POS_Y = MIN_POS;
    double intermed = (MAX_POS - MIN_POS) / 2;
    boolean buttonL1 = gamepad2.left_bumper;

    @Override
    public void init() {
        telemetry.addData("Status", "TeleOp_Started");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
    }

    void motorPower(float powFL, float powBL, float powFR, float powBR) {
        FL.setPower(powFL);
        BL.setPower(powBL);
        FR.setPower(powFR);
        BR.setPower(powBR);

    }

    @Override
    public void start() {
        runtime.reset();
    }


    public void climbOpMode(){
      servo = hardwareMap.get(Servo.class, "right_hand");
      servo.setPosition(Servo.MIN_POSITION);

      telemetry.addData(">", "Press Start to scan Servo");
      telemetry.update();

      while (opModeIsActive()){
          if(buttonY){
              position += INCREMENT;
              POS_Y += INCREMENT;

              if(position >= MAX_POS){
                  POS_Y = MAX_POS;
                  position = MAX_POS;
                  Up = !Up;
              }
          }
      }

      telemetry.addData("Servo Position", "%5.2f", position);
      telemetry.addData(">", "Press Stop to stop");
      telemetry.update();

      servo.setPosition(position);
      servo.setPosition(POS_Y);
      sleep(CYCLE_MS);
      idle();

      telemetry.addData(">", "Done");
      telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "TeleOp_Executed: " + runtime.toString());
        telemetry.update();

        frontalMotor();
        backMotor();
        adjustMotor();
        round();
        roundX();
        axisXY();
    }

    private void adjustMotor() {
        if (gamepad1.dpad_up) {
            motorPower(0.65f, 0.65f, 0.65f, 0.65f);
        }
        if (gamepad1.dpad_down) {
            motorPower(-0.65f, -0.65f, -0.65f, -0.65f);
        }
        if (gamepad1.dpad_right) {
            motorPower(0.65f, 0.65f, 0.65f, 0.65f);
        }
        if (gamepad1.dpad_left) {
            motorPower(-0.65f, -0.65f, -0.65f, -0.65f);
        }
    }

    private void backMotor() {
        motorPower(0, /*powBL*/ gamepad1.left_trigger, 0, /*powBR*/ gamepad1.left_trigger);
    }

    private void frontalMotor() {
        motorPower(/*powFL*/ -gamepad1.right_trigger, 0, /*powFR*/ -gamepad1.right_trigger, 0);
    }


    private void axisXY() {
        //Axis X
        motorPower(-gamepad1.right_stick_x, -gamepad1.right_stick_x, gamepad1.right_stick_x, gamepad1.right_stick_x);

        //Axis Y
        motorPower(-gamepad1.left_stick_y, -gamepad1.right_stick_y, gamepad1.right_stick_y, gamepad1.right_stick_y);

    }

    private void round(){
        if (gamepad1.right_bumper){
            motorPower(-1, -1, 1, 1);
        }
        if (gamepad1.left_bumper){
            motorPower(1, 1, -1, -1);
        }
    }

    private void roundX(){
        if (gamepad1.x){
                motorPower(0, 1, 0, 1);
        }
        if (gamepad1.b){
            motorPower(1, 0, 1, 0);
        }
    }


    @Override
    public void stop(){
        telemetry.addData("Status", "TeleOp_Final");
    }
}

