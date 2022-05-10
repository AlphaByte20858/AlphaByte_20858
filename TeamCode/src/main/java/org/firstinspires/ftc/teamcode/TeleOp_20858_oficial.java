package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp_Oficial_20858", group = "OpMode")
public class TeleOp_20858_oficial extends OpMode{
    private final ElapsedTime runtime = new ElapsedTime();
    public DcMotor mFE, mTE, mFD, mTD;


    public boolean active = false;
    public DcMotor motorHex;

    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position

    Servo servoHand;
    Servo servo;
    double position = MIN_POS;
    double POS_Y = MIN_POS;


    @Override
    public void init() {
        mFE = hardwareMap.get(DcMotor.class, "FL");
        mFD = hardwareMap.get(DcMotor.class, "FR");
        mTE = hardwareMap.get(DcMotor.class, "BL");
        mTD = hardwareMap.get(DcMotor.class, "BR");

        servo = hardwareMap.get(Servo.class, "arm");
        servoHand = hardwareMap.get(Servo.class, "hand");


        mFE.setDirection(DcMotor.Direction.REVERSE);
        mFD.setDirection(DcMotor.Direction.FORWARD);
        mTE.setDirection(DcMotor.Direction.REVERSE);
        mTD.setDirection(DcMotor.Direction.FORWARD);

        mTE.getCurrentPosition();
        mTD.getCurrentPosition();
        mFE.getCurrentPosition();
        mFD.getCurrentPosition();
    }

    void motorPower(float powLF, float powLB, float powRF, float powRB){
        mFE.setPower(powLF);
        mTE.setPower(powLB);
        mFD.setPower(powRF);
        mTD.setPower(powRB);
    }


    @Override
    public void loop() {

        //telemetry.addData(">", "Press Start to scan Servo.");
        telemetry.update();

        telemetry.addData("Status", "TeleOp sendo executado: " + runtime.toString());
        telemetry.addData("Status", "MotorPower: " + mFE);
        telemetry.addData("Status", "MotorPower: " + mFD);
        telemetry.addData("Status", "MotorPower: " + mTE);
        telemetry.addData("Status", "MotorPower: " + mTD);


        drive();
        arm();
        roda();
    }

    private void roda() {
        if(gamepad2.x && !active)
        {
            active = true;
            motorHex.setPower(0.5);
        }
        if(gamepad2.a && active)
        {
            motorHex.setPower(0);
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
        telemetry.addData("status", "Executando");
        servo.setPosition(MIN_POS);
        servoHand.setPosition(MIN_POS);

        if (gamepad2.y) {
            servo.setPosition(MAX_POS);
            servoHand.setPosition(MAX_POS);
        }
        if(gamepad2.y){
            servo.setPosition(MIN_POS);
            servoHand.setPosition(MIN_POS);
        }


        telemetry.addData("Servo Position", "%5.2f", position);
        telemetry.addData(">", "Press Stop to end test." );
        telemetry.update();

        // Set the servo to the new position and pause;
        servo.setPosition(position);
        servo.setPosition(POS_Y);

        telemetry.addData(">", "Done");
        telemetry.update();
    }

}