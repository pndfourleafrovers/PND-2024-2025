package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.HardwareMap;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Helper.EncoderMovement;

@Autonomous(name="Encoder Autonomous", group="Autonomous")
@Disabled

public class EncoderAutoTest extends LinearOpMode {

    public EncoderMovement encoderMovement;

    @Override
    public void runOpMode() {

        encoderMovement = new EncoderMovement(hardwareMap);

        encoderMovement.driveForward(10,0.5);
        encoderMovement.driveBackward(10,0.5);
        encoderMovement.turnToHeading(90);
        encoderMovement.strafeLeft(5,0.5);
        encoderMovement.strafeRight(5,0.5);
    }



}
