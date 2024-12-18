package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoTestRoute1", group = "Routes")
public class AutoTestRoute1 extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(0, 0, 0); // Start position

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Action route1 = drive.actionBuilder(startPose)
                .lineToY(1.761) // Move 3 tiles forward (3 * 587mm ≈ 1.761m)
                .strafeTo(new Vector2d(0.2935, 1.761)) // Strafe 0.5 tiles right (0.5 * 587mm ≈ 0.2935m)
                .build();

        telemetry.addLine("Route 1 Ready: 3 tiles forward, 0.5 right.");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(route1);

        telemetry.addLine("Route 1 Complete.");
        telemetry.update();
    }
}
