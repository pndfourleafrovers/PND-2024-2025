package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Helper.Arm;
import org.firstinspires.ftc.teamcode.Helper.EncoderMovement;
import org.firstinspires.ftc.teamcode.Helper.SMM;
import org.firstinspires.ftc.teamcode.Helper.Viper;

@Autonomous(name="Final Auto", group="Autonomous")
@Disabled

public class FinalAuto extends LinearOpMode {

    public EncoderMovement encoderMovement;
    private org.firstinspires.ftc.teamcode.Helper.Viper Viper;
    private org.firstinspires.ftc.teamcode.Helper.SMM SMM;
    private org.firstinspires.ftc.teamcode.Helper.Arm Arm;
    @Override
    public void runOpMode() {
        Viper = new Viper(hardwareMap);
        SMM = new SMM(hardwareMap);
        Arm = new Arm(hardwareMap);
        encoderMovement = new EncoderMovement(hardwareMap);

        encoderMovement.driveForward(10,0.5);
        encoderMovement.driveBackward(10,0.5);
        encoderMovement.turnToHeading(90);
        encoderMovement.strafeLeft(5,0.5);
        encoderMovement.strafeRight(5,0.5);

        Viper.viperMovementSetSpeed(20, 0.7);
        sleep(2000);
        Arm.armMovement(30);
    }



}