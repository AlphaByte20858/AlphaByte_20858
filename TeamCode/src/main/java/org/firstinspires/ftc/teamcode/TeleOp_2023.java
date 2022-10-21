package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name=" TeleOp_2023", group="Opmode")
@Disabled

public class TeleOp_2023 extends OpMode{

    public DcMotor MotorEsquerdoFrente, MotorEsquerdoTras, MotorDireitoFrente, MotorDireitoTras;
    public DcMotor coreHex1;
    public Servo Servo1;

    @Override
    public void init() {
        MotorEsquerdoFrente = hardwareMap.get(DcMotor.class, "MEF");
        MotorEsquerdoTras = hardwareMap.get(DcMotor.class, "MET");
        MotorDireitoFrente = hardwareMap.get(DcMotor.class, "MDF");
        MotorDireitoTras = hardwareMap.get(DcMotor.class, "MDT");

        coreHex1 = hardwareMap.get(DcMotor.class, "coreHex1");
        coreHex1.setDirection(DcMotor.Direction.FORWARD);

        Servo1 = hardwareMap.get(Servo.class, "servo1");

        MotorEsquerdoFrente.setDirection(DcMotor.Direction.FORWARD);
        MotorEsquerdoTras.setDirection(DcMotor.Direction.REVERSE);
        MotorDireitoFrente.setDirection(DcMotor.Direction.FORWARD);
        MotorDireitoTras.setDirection(DcMotor.Direction.REVERSE);

        MotorEsquerdoFrente.getCurrentPosition();
        MotorEsquerdoTras.getCurrentPosition();
        MotorDireitoFrente.getCurrentPosition();
        MotorDireitoTras.getCurrentPosition();
    }

    @Override
    public void loop() {
        Move(5,5,5,5);
        CoreHex();
        ServoGarra();
    }

    public void Move(double ForcaMEF, double ForcaMET, double ForcaMDF, double ForcaMDT)
    {
        double max;

        double axial   = -gamepad1.left_stick_y;
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        ForcaMEF  = axial + lateral + yaw;
        ForcaMET = axial - lateral - yaw;
        ForcaMDF   = axial - lateral + yaw;
        ForcaMDT  = axial + lateral - yaw;

        MotorEsquerdoFrente.setPower(ForcaMEF);
        MotorDireitoTras.setPower(ForcaMET);
        MotorDireitoFrente.setPower(ForcaMDF);
        MotorDireitoTras.setPower(ForcaMDT);
    }

    public void CoreHex()
    {
        double up = 0;
        double down = 0;
        double increment = 0.1;
        double maxPosicao = 1;
        double minPosicao = 0.1;

        boolean isUp  = gamepad2.right_bumper;
        boolean isDown = gamepad2.left_bumper;

        if(isUp)
        {
            up += increment;
            if(up >= increment)
            {
                up = maxPosicao;
            }
        }
        if (isDown)
        {
            down -= increment;
            if(down >= increment)
            {
                down = minPosicao;
            }
        }
    }

    public void ServoGarra()
    {
        boolean isServo = gamepad2.b;
        if(isServo)
        {
            Servo1.setPosition(1);
        }
    }
}
