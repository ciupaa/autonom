package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "AutoTest", group = "Robot")
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Define the starting position
        Pose2d initialPose = new Pose2d(0, 0, 0);

        // Initialize the MecanumDrive
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Define the forward trajectory: move forward by 1 meter
        Action forwardAction = drive.actionBuilder(initialPose)
                .lineToY(1.0) // Move forward by 1 meter
                .build();

        // Define the strafe trajectory: strafe right by 0.5 meters
        Action strafeRightAction = drive.actionBuilder(initialPose) // Start again from the initial pose
                .strafeTo(new Vector2d(0.5, 1.0)) // Move to X=0.5, Y=1.0
                .build();

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Run the actions sequentially
        Actions.runBlocking(
                new SequentialAction(
                        forwardAction,
                        strafeRightAction
                )
        );

        telemetry.addLine("Autonomous Routine Complete.");
        telemetry.update();
    }
}
