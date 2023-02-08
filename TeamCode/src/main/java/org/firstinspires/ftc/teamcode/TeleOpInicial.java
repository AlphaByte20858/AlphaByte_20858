
/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="TesteTO", group="OpMode")
public class TeleopInicial extends OpMode{
    // criação das variáveis para os motores de movimentação
    // do sistema linear e também do servo motor
    public Servo servoMotor = null;
    public DcMotor motorEsquerdoF, motorEsquerdoT, motorDireitoF, motorDireitoT = null;
    public DcMotor Arm = null;

    @Override
    public void init(){


        // Nesta parte do codigo nos classicamos as variaveis
        // dos motores de movimento
        motorEsquerdoF  = hardwareMap.get(DcMotor.class, "LeftDriveUp");
        motorDireitoF  = hardwareMap.get(DcMotor.class, "RightDriveUp");
        motorEsquerdoT = hardwareMap.get(DcMotor.class, "LeftDriveDown");
        motorDireitoT = hardwareMap.get(DcMotor.class, "RightDriveDown");

        // E nesta parte classificamos o motor do sistema linear
        Arm = hardwareMap.get(DcMotor.class, "Arm");

        // E aqui o motor servo
        servoMotor = hardwareMap.get(Servo.class, "Servo");


        // Em seguida nos indentificamos as direções
        // dos motores de movimento

        motorEsquerdoF.setDirection(DcMotor.Direction.REVERSE);
        motorDireitoF.setDirection(DcMotor.Direction.FORWARD);
        motorEsquerdoT.setDirection(DcMotor.Direction.REVERSE);
        motorDireitoT.setDirection(DcMotor.Direction.FORWARD);

        // E também classificamos o motor do sistema linear

        Arm.setDirection(DcMotor.Direction.FORWARD);



    }

    // aqui criamos a função loop
    public void loop(){
        // Uso das funções criadas no loop
        servo();
        mover();
        linear();
    }

    //Criação da função da movimentação

    public void mover(){

        // Criação das váriaveis de força para a movimentação
        double axial   = gamepad1.right_trigger - gamepad1.left_trigger;
        double lateral = gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        // Criação de váriaveis para a movimentação em 8 direções
        double absaxial = Math.abs(axial);
        double abslateral = Math.abs(lateral);
        double absyaw= Math.abs(yaw);
        double denominador = Math.max(absaxial + abslateral + absyaw, 1);

        // Calculos para a movimentação das rodas omnidirectionais mecanum
        double motorEsquerdoFf = (axial + lateral + yaw / denominador);
        double motorDireitoFf = (axial - lateral - yaw / denominador);
        double motorEsquerdoTf = (axial - lateral + yaw / denominador);
        double motorDireitoTf = (axial + lateral - yaw / denominador);

        // Função que define que as Forças utilizadas em cada motor

        allMotorsPower(motorEsquerdoFf, motorDireitoFf, motorEsquerdoTf, motorDireitoTf);


        // Nessa sequencia de if nos selecionamos as setas dos controles como meios
        // de movimentação mais precisa, sendo mais lentas e tendo um controle em ambas as 8 direções

        // Seta de baixo
        if(gamepad1.dpad_down){
            allMotorsPower(-0.35,-0.35,-0.35,-35);
        }

        // Seta de cima
        if(gamepad1.dpad_up){
            allMotorsPower(0.35,0.35,0.35,0.35);
        }

        // Seta da direita
        if(gamepad1.dpad_right){
            allMotorsPower(0.35,-0.35,-0.35,0.35);
        }

        // Seta da esquerda
        if(gamepad1.dpad_left){
            allMotorsPower(-0.35,0.35,0.35,-0.35);
        }


        // Nestas linhas de códigos nós usamos o comando Telemetry para
        // Conseguirmos ver as forças dos motores de movimentação no Drive Hub

        telemetry.addData("A potencia do motorEsquerdoF é de:", motorEsquerdoFf);
        telemetry.addData("A potencia do motorDireitoF é de:", motorDireitoFf);
        telemetry.addData("A potencia do motorEsquerdoT é de:", motorEsquerdoTf);
        telemetry.addData("A potencia do motorDireitoT é de:", motorDireitoTf);

    }

    // Criação da Função que define o poder de todos
    // os motores utilizados na movimentação

    public void allMotorsPower(double paMEF, double paMDF, double paMET, double paMDT){
        motorEsquerdoF.setPower(paMEF);
        motorDireitoF.setPower(paMDF);
        motorEsquerdoT.setPower(paMET);
        motorDireitoT.setPower(paMDT);
    }
    //Criação da função do sistema linear

    public void linear() {
        // Criação das variáveis utilizadas para definir as forças
        // utilizadas no uso do sistema linear
        boolean poderCima = gamepad2.right_bumper;
        boolean poderBaixo = gamepad2.left_bumper;
        boolean poderImovel = gamepad2.x;
        double pow;

        // Nessa sequencia de if, nos colocamos os poderes para elevar
        // o sistema linear, para abaixar o sistema linear e também
        // para o ele estar imovel no ar

        // Para subir o sistema linear

        if(poderCima){
            pow = -0.6;
            Arm.setPower(pow);
        }

        // Para descer o sistema linear

        if(poderBaixo){
            pow = 0.3;
            Arm.setPower(pow);
        }

        // Para deixar o sistema linear imovel no ar

        if(poderImovel){
            pow = -0.2;
            Arm.setPower(pow);
        }

        // Para deixar o sistema linear sem poder caso nenhum dos botões seja apertado

        else{
            pow = 0;
            Arm.setPower(pow);
        }

        // Função que permite que o poder do motor do sistema linear
        // seja exibido no Drive Hub
        telemetry.addData("A potencia do motor do sistema linear é de", pow);

    }

    // Criação da váriavel de poder do servo fora da função servo
    // para assim não ser necessário pressionar o botão para definir a força
    // e sim só apenas apertar

    double powServo = 0;

    //Criação da função do servo
    public void servo() {

        // Criação das varíaveis tanto de força quanto dos botões
        // utilizados para abrir e fechar o servo

        boolean poderAberto = gamepad2.a;
        boolean poderFechado = gamepad2.b;
        double aberto = 0;
        double fechado = 1;

        // Função que define o servo estar aberto

        if (poderAberto) {
            powServo = aberto;
            servoMotor.setPosition(powServo);
        }

        // Função que define o servo estar fechado

        else if (poderFechado) {
            powServo = fechado;
            servoMotor.setPosition(powServo);
        }

        // Função que mostra o poder do servo no Drive Hub
        telemetry.addData("A potencia do motor do servo é de:", powServo);
    }
}
