package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Autonomous", group = "LinearOpMode")
@Disabled
public class Autonomous_20858 extends LinearOpMode {

    public DcMotor fL, fR, bL, bR;
    private int pFL, pFR, pBL, pBR;


    @Override
    public void runOpMode(){
        fL = hardwareMap.get(DcMotor.class, "FL");
        fR = hardwareMap.get(DcMotor.class, "FR");
        bL = hardwareMap.get(DcMotor.class, "BL");
        bR = hardwareMap.get(DcMotor.class, "BR");
        /////////////////////////////////////////////////////////
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /////////////////////////////////////////////////////////
        fL.setDirection(DcMotor.Direction.REVERSE);
        fR.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.FORWARD);
    }

    /*******************************
     * move robot
     * @param tfL - left front target
     * @param tbL - back left target
     * @param tfR - right front target
     * @param tbR - back right target
     * @param velFL - velocityFL
     * @param velFR - velocityFR
     * @param velBL - velocityBL
     * @param velBR - velocityBR
     */
    public void moveRobot(double tfL, double tbL, double tfR, double tbR, double velFL, double velFR, double velBL, double velBR){
        pFL += tfL;
        pFR += tfR;
        pBL += tbL;
        pBR += tbR;

        fL.setTargetPosition(pFL);
        fR.setTargetPosition(pFR);
        bL.setTargetPosition(pBL);
        bR.setTargetPosition(pBR);

        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fL.setPower(velFL);
        fR.setPower(velFR);
        bL.setPower(-velBL);
        bR.setPower(-velBR);

        while(opModeIsActive() && fL.isBusy() && fR.isBusy() && bL.isBusy() && bR.isBusy()){
            idle();
        }
    }
}
