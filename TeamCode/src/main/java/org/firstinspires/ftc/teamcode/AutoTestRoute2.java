package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoTestRoute2", group = "Routes")
public class AutoTestRoute2 extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(0, 0, 0); // Start position

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Action route2 = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(0.2935, 0.0)) // Strafe 0.5 tiles to the right
                .lineToY(1.761) // Move 3 tiles forward (3 * 587mm ≈ 1.761m)
                .strafeTo(new Vector2d(0.0, 1.761)) // Strafe 0.5 tiles left
                .build();

        telemetry.addLine("Route 2 Ready: 0.5 right, 3 forward, 0.5 left.");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(route2);

        telemetry.addLine("Route 2 Complete.");
        telemetry.update();
    }
}
