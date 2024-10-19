package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Helper.OdometryMovement;
@Autonomous(name="OdometryAutonomous", group="Autonomous")
@Disabled
public class TEST2 extends LinearOpMode {

    private OdometryMovement odometryMovement;

    @Override
    public void runOpMode() {
        // Initialize the OdometryMovement class
        odometryMovement = new OdometryMovement(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Example movement
        odometryMovement.moveToPosition(24, 0, 0.5); // Move forward 24 inches
        odometryMovement.moveToPosition(24, 24, 0.5); // Strafe right 24 inches
        odometryMovement.rotateToHeading(90, 0.3);   // Turn 90 degrees clockwise
        odometryMovement.moveToPosition(0, 24, 0.5); // Move backward 24 inches
    }
}
