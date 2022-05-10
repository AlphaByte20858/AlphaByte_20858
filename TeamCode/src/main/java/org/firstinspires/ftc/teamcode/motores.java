package TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Motores_TeleOp", group = "TeleOp")
@Disabled
public class motores extends OpMode {
    public DcMotor mFE, mTE, mFD, mTD;
    /**************************************************************************
     * mFE - Variável motor frente esquerda                                   *
     * mTE - Variável motor tras esquerda                                     *
     * mFD - Variável motor frente direita                                    *
     * mTD - Variável motor tras direita                                      *
     **************************************************************************/

    @Override
    public void init() {
        mFE = hardwareMap.get(DcMotor.class, "FL");
        mFD = hardwareMap.get(DcMotor.class, "FR");
        mTE= hardwareMap.get(DcMotor.class, "BL");
        mTD = hardwareMap.get(DcMotor.class, "BR");

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

    }
}
