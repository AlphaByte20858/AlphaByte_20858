package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disable;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Move_Encoder", group = "Auto")
public class Move_Encoder_Autonomous extends LinearOpMode{
    public DcMotor fL, fR, bL, bR;
    private int pFL, pFR, pBL, pBR;

    public void runOpMode(){
        fL = hardwareMap.get(DcMotor.Class, "mFL");
        fR = hardwareMap.get(DcMotor.Class, "mFR");
        bL = hardwareMap.get(DcMotor.Class, "mBL");
        bR = hardwareMap.get(DcMotor.Class, "mBR");

        fL.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);
        
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    /*********************************************
         * parameters to Autonomous
         * 
         * @param tFL
         * @param tFR
         * @param tBL
         * @param tBR
         * @param vel
         * 
    **********************************************/

    public void moveBot(int tFL, int tFR, int tBL, int tBR, float vel){
        pFL += tFL;
        pFR += tFR;
        pBL += tBL;
        pBR += tBR;
        
        fL.setTargetPosition(pFL);
        fR.setTargetPosition(pFR);
        bL.setTargetPosition(pBL);
        bR.setTargetPosition(pBR);

        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fL.setPower(vel);
        fR.setPower(vel);
        bL.setPower(vel);
        bR.setPower(vel);

        while(opModeIsActive() && fL.isBusy() && fR.isBusy() && bL.isBusy() && bR.isBusy()){
            idle();
        }
    }

}