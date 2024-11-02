package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Helper.Drivetrain;
import org.firstinspires.ftc.teamcode.Components.SharedData;

@Autonomous(name = "Autonomous Drive Sequence", group = "Linear Opmode")
@Disabled
public class AutonomousMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        SharedData sharedData = new SharedData();
        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry, sharedData);

        waitForStart();

        // Calling movement methods with specified power
        drivetrain.driveForward(5, 0.3);    // Drive forward 5 inches at 50% power
        drivetrain.strafeLeft(5, 0.3);      // Strafe left 5 inches at 40% power
        drivetrain.turnToHeading(360, 0.3); // Spin 360 degrees at 30% power

        telemetry.addData("Status", "Autonomous Sequence Complete");
        telemetry.update();
        sleep(2000); // Optional delay to view the status
    }
}
