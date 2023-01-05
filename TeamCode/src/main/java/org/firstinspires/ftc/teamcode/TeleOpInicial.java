ackage org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="TeleOpInicial", group="OpMode")
public class TeleOpInicial extends OpMode{
    public Servo servoMotor;
    public DcMotor motorEsquerdoF, motorEsquerdoT, motorDireitoF, motorDireitoT = null;
    public DcMotor Arm;

    @Override
    public void init(){
        motorEsquerdoF  = hardwareMap.get(DcMotor.class, "leftDriveUp");
        motorDireitoF  = hardwareMap.get(DcMotor.class, "RightDriveUp");
        motorEsquerdoT = hardwareMap.get(DcMotor.class, "LeftDriveDown");
        motorDireitoT = hardwareMap.get(DcMotor.class, "RightDriveDown");

        motorEsquerdoF.setDirection(DcMotor.Direction.FORWARD);
        motorDireitoF.setDirection(DcMotor.Direction.FORWARD);
        motorEsquerdoT.setDirection(DcMotor.Direction.REVERSE);
        motorDireitoT.setDirection(DcMotor.Direction.REVERSE);

        Arm = hardwareMap.get(DcMotor.class, "Arm");

        Arm.setDirection(DcMotor.Direction.FORWARD);

        servoMotor = hardwareMap.get(Servo.class, "Arm");

        resetRuntime();


    }

    public void loop(){
        linear();
        servo();
        mover();
    }

    public void mover(){
        double axial   = -gamepad1.left_stick_y;
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        double denominador = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
        double motorEsquerdoFt  = axial + lateral + yaw / denominador;
        double motorDiretoFt = axial - lateral - yaw / denominador;
        double motorEsquerdoTt   = axial - lateral + yaw / denominador;
        double motorDireitoTt  = axial + lateral - yaw / denominador;

        allMotorsPower(motorEsquerdoFt,motorDiretoFt,motorEsquerdoTt,motorDireitoTt);

        telemetry.addData("A potencia do motorEsquerdoF é de:", motorEsquerdoFt);
        telemetry.addData("A potencia do motorDireitoF é de:", motorDiretoFt);
        telemetry.addData("A potencia do motorEsquerdoF é de:", motorEsquerdoTt);
        telemetry.addData("A potencia do motorEsquerdoF é de:", motorDireitoTt);
        telemetry.addData("O tempo em que o robô opera é de :", getRuntime());
    }
    public void linear() {
        boolean poderCima = gamepad2.right_bumper;
        boolean poderBaixo = gamepad2.left_bumper;
        double pow = 0;


        if (poderCima) {
            pow = pow + 0.1;
            Arm.setPower(pow);
            if (pow >= 1) {
                Arm.setPower(1);
            }
        }
        if (poderBaixo) {
            pow = pow - 0.1;
            Arm.setPower(pow);
            if(pow <= -1);
            Arm.setPower(-1);

            telemetry.addData("A potencia do motor do sistema linear é de", pow);


        }
    }
    public void servo() {
        boolean poderAberto = gamepad2.a;
        boolean poderFechado = gamepad2.b;
        double powServo;
        double aberto = 0;
        double fechado = 1;

        if (poderAberto) {
            powServo = aberto;
            servoMotor.setPosition(powServo);

        } else if (poderFechado) {
            powServo = fechado;
            servoMotor.setPosition(powServo);

            telemetry.addData("A potencia do motor do servo é de:", powServo);
        }

    }

    public void allMotorsPower(double paMEF, double paMDF, double paMET, double paMDT){
        motorEsquerdoF.setPower(paMEF);
        motorDireitoF.setPower(paMDF);
        motorEsquerdoT.setPower(paMET);
        motorDireitoT.setPower(paMDT);
    }


}
