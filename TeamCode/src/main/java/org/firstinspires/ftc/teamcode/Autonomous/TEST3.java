package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;


@Autonomous(name = "Autonomous Drive Sequence", group = "Linear Opmode")
public class TEST3 extends LinearOpMode {

    private TrajectorySequenceBuilder SequenceBuilder;

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
     //   SequenceBuilder = new TrajectorySequenceBuilder(Pose2d, Double, TrajectoryVelocityConstraint,
       //         TrajectoryAccelerationConstraint,baseTurnConstraintMaxAngVel,baseTurnConstraintMaxAngAccel
        );








    }





}
