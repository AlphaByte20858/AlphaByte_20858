package org.firstinspires.ftc.teamcode.AUTO;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AUTO.AprilTagAutonomousInitDetectionExample;
import org.openftc.apriltag.AprilTagDetection;


@Autonomous(name = "AutonomoTeste1", group = "LinearOpMode")

public class autonomous extends LinearOpMode {
    public AprilTagAutonomousInitDetectionExample cam = new AprilTagAutonomousInitDetectionExample();
    public DcMotor motorEsquerdoF, motorEsquerdoT, motorDireitoF, motorDireitoT = null;
    BNO055IMU imu;
    DcMotorEx MDF,MEF,MDT,MET = null;

    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGains = new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients pidCoeffs1 = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGains1 = new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients pidCoeffs2 = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGains2 = new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients pidCoeffs3 = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGains3 = new PIDCoefficients(0, 0, 0);


    double integral = 0;
    double error = 0;
    double currvel = 0;
    double derivate = 0;
    double deltaError = 0;
    private double lastError = 0;

    double integral1 = 0;
    double error1 = 0;
    double currvel1 = 0;
    double derivate1 = 0;
    double deltaError1 = 0;
    private double lastError1 = 0;

    double integral2 = 0;
    double error2 = 0;
    double currvel2 = 0;
    double derivate2 = 0;
    double deltaError2 = 0;
    private double lastError2 = 0;

    double integral3 = 0;
    double error3 = 0;
    double currvel3 = 0;
    double derivate3 = 0;
    double deltaError3 = 0;
    private double lastError3 = 0;


    ElapsedTime tempo = new ElapsedTime();


    @Override
    public void runOpMode() {

        MEF  = hardwareMap.get(DcMotorEx.class, "LeftDriveUp");
        MDF  = hardwareMap.get(DcMotorEx.class, "RightDriveUp");
        MET = hardwareMap.get(DcMotorEx.class, "LeftDriveDown");
        MDT = hardwareMap.get(DcMotorEx.class, "RightDriveDown");

        modemoto(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        modemoto(DcMotor.RunMode.RUN_USING_ENCODER);

        MEF.setDirection(DcMotor.Direction.REVERSE);
        MDF.setDirection(DcMotor.Direction.FORWARD);
        MET.setDirection(DcMotor.Direction.REVERSE);
        MDT.setDirection(DcMotor.Direction.FORWARD);


        resetRuntime();
        waitForStart();

        if (opModeIsActive()) {

            MEF.setTargetPosition(500);
            MDF.setTargetPosition(500);
            MET.setTargetPosition(500);
            MDT.setTargetPosition(500);
            PIDVelMoveAll(0.6);
            modemoto(DcMotor.RunMode.RUN_TO_POSITION);


            while (opModeIsActive() && motorDireitoF.isBusy() && motorDireitoT.isBusy() && motorEsquerdoF.isBusy() && motorEsquerdoT.isBusy()) {
                idle();

            }
        }
    }
    public double MDFpid(double velocidade) {


        currvel = MDF.getVelocity();

        error = velocidade - currvel;

        integral += error * tempo.seconds();

        deltaError = (error - lastError);

        derivate = deltaError / tempo.seconds();

        lastError = error;

        pidGains.p = error * pidCoeffs.p;
        pidGains.i = integral * pidCoeffs.i;
        pidGains.d = derivate * pidCoeffs.d;

        tempo.reset();

        double output = (pidGains.p + pidGains.i + pidGains.d + velocidade);
        return output;
    }
    public double MEFpid(double velocidade1) {


        currvel1 = MEF.getVelocity();

        error1 = velocidade1 - currvel1;

        integral1 += error1 * tempo.seconds();

        deltaError1 = (error1 - lastError1);

        derivate1 = deltaError1 / tempo.seconds();

        lastError1 = error1;

        pidGains1.p = error1 * pidCoeffs1.p;
        pidGains1.i = integral1 * pidCoeffs1.i;
        pidGains1.d = derivate1 * pidCoeffs1.d;

        tempo.reset();

        double output = (pidGains1.p + pidGains1.i + pidGains1.d + velocidade1);
        return output;
    }
    public double METpid(double velocidade2) {


        currvel2 = MET.getVelocity();

        error2 = velocidade2 - currvel2;

        integral2 += error2 * tempo.seconds();

        deltaError2 = (error2 - lastError2);

        derivate2 = deltaError2 / tempo.seconds();

        lastError2 = error2;

        pidGains2.p = error * pidCoeffs2.p;
        pidGains2.i = integral * pidCoeffs2.i;
        pidGains2.d = derivate * pidCoeffs2.d;

        tempo.reset();

        double output = (pidGains2.p + pidGains2.i + pidGains2.d + velocidade2);
        return output;
    }
    public double MDTpid(double velocidade3) {


        currvel3 = MDT.getVelocity();

        error3 = velocidade3 - currvel3;

        integral3 += error3 * tempo.seconds();

        deltaError3 = (error3 - lastError3);

        derivate3 = deltaError3 / tempo.seconds();

        lastError3 = error3;

        pidGains3.p = error3 * pidCoeffs3.p;
        pidGains3.i = integral3 * pidCoeffs3.i;
        pidGains3.d = derivate3 * pidCoeffs3.d;

        tempo.reset();

        double output = (pidGains3.p + pidGains3.i + pidGains3.d + velocidade3);
        return output;
    }
    public void PIDVelMoveAll(double speed){
        allMotorsPower(MEFpid(speed), MDFpid(speed), METpid(speed), MDTpid(speed));
    }
    public void PIDVelMove(double paMEF1, double paMDF1, double paMET1, double paMDT1 ){
        allMotorsPower(MEFpid(paMEF1), MDFpid(paMDF1), METpid(paMET1), MDTpid(paMDT1));
    }
    public void allMotorsPower(double paMEF, double paMDF, double paMET, double paMDT){
        MEF.setPower(paMEF);
        MDF.setPower(paMDF);
        MET.setPower(paMET);
        MDT.setPower(paMDT);
    }
    public void modemoto(DcMotor.RunMode mode){
        MDF.setMode(mode);
        MDT.setMode(mode);
        MEF.setMode(mode);
        MET.setMode(mode);
    }
    }
