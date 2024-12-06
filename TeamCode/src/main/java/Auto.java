package org.firstinspires.ftc.teamcode.AUTONOMI.BLUE;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import  com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareMapping;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Variables.DefVal;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Blue_Dreapta",group="Iuliu")//Lidi mu pune mana
public class Blue_Dreapta extends LinearOpMode {
    Pose2d beginPose = new Pose2d(-36, 60, Math.toRadians(270));
    Pose2d cPose;
    HardwareMapping robot = new HardwareMapping();
    OpenCvCamera externalCamera;
    HardwareMapping.Intake intake = robot.new Intake();
    HardwareMapping.Outtake outtake = robot.new Outtake();
    Servo intakeServoRight,intakeServoLeft;
    Blue_Dreapta.nume pipeline;
    String PropZone ="RIGHT";

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        intakeServoLeft=hardwareMap.get(Servo.class,"intakeServoLeft");
        intakeServoRight=hardwareMap.get(Servo.class,"intakeServoRight");
        intakeServoLeft.setPosition(DefVal.iLevel6);
        intakeServoRight.setPosition(DefVal.iLevel6);

        // Initialize the camera and pipeline
        initExternalCamera();

        while (!isStarted() && !isStopRequested())
        {
            PropZone = pipeline.isPointInsideRect();

            if (pipeline.isPointInsideRect()=="LEFT") {
                telemetry.addData("LEFT","");
            } else if(pipeline.isPointInsideRect()=="RIGHT"){
                telemetry.addData("RIGHT","");
            }else if(pipeline.isPointInsideRect()=="MIDDLE") {
                telemetry.addData("MIDDLE","");
            }

            telemetry.update();
        }
        Action LeftLine= drive.actionBuilder(beginPose) //stanga
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-26,30,Math.toRadians(270)),Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-30,40,Math.toRadians(270)),Math.toRadians(270))
                .turn(Math.toRadians(100))
                //stack
                .setTangent(Math.toRadians(-180))
                //.setReversed(false)
                .splineToLinearHeading(new Pose2d(-60.04,36,Math.toRadians(0)),Math.toRadians(-180))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(0)), Math.toRadians(-90))
                //se duce la backboard
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(20, 12, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(48,30,Math.toRadians(0)),Math.toRadians(0))
                //.strafeLeft(25)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(20, 12, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-60,12,Math.toRadians(0)),Math.toRadians(180))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(20, 12, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(48,30,Math.toRadians(0)),Math.toRadians(0))

                .waitSeconds(0.1)
                //.setTangent(Math.toRadians(180))
                //.splineToLinearHeading(new Pose2d(-80,-58,Math.toRadians(90)),Math.toRadians(180))
                .build();

        Action MiddleLine = drive.actionBuilder(beginPose) //mijloc
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-43,30,Math.toRadians(270)),Math.toRadians(270))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-34.5,40,Math.toRadians(270)),Math.toRadians(-270))
                .waitSeconds(0.1)
                //.setTangent(Math.toRadians(180))
                //.splineToLinearHeading(new Pose2d(-80,-58,Math.toRadians(90)),Math.toRadians(180))
                .turn(Math.toRadians(100))
                //stack
                .setTangent(Math.toRadians(-180))
                //.setReversed(false)
                .splineToLinearHeading(new Pose2d(-60.04,36,Math.toRadians(0)),Math.toRadians(-180))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-58.28, 12, Math.toRadians(0)), Math.toRadians(-90))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(20, 12, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(48,30,Math.toRadians(0)),Math.toRadians(0))
                //.strafeLeft(25)
                .setTangent(Math.toRadians(-140))
                .splineToLinearHeading(new Pose2d(-60.04,12,Math.toRadians(0)),Math.toRadians(-180))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(20, 12, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(48,30,Math.toRadians(0)),Math.toRadians(0))

                .waitSeconds(0.1)
                //.setTangent(Math.toRadians(180))
                //.splineToLinearHeading(new Pose2d(-80,-58,Math.toRadians(90)),Math.toRadians(180))
                .build();

        Action RightLine=drive.actionBuilder(beginPose) //dreapta
                .setReversed(true)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-43, 33, Math.toRadians(250)), Math.toRadians(-90))
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(-38.28, 45.7, Math.toRadians(250)), Math.toRadians(90))
                //.setTangent(Math.toRadians(90))
                .turn(Math.toRadians(100))
                //stack
                .setTangent(Math.toRadians(-180))
                //.setReversed(false)
                .splineToLinearHeading(new Pose2d(-60.04,36,Math.toRadians(0)),Math.toRadians(-180))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-58.28, 12, Math.toRadians(0)), Math.toRadians(-90))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(20, 12, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(48,30,Math.toRadians(0)),Math.toRadians(0))
                //.strafeLeft(25)
                .setTangent(Math.toRadians(-140))
                .splineToLinearHeading(new Pose2d(-60.04,12,Math.toRadians(0)),Math.toRadians(-180))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(20, 12, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(48,30,Math.toRadians(0)),Math.toRadians(0))

                .waitSeconds(0.1)
                //.setTangent(Math.toRadians(180))
                //.splineToLinearHeading(new Pose2d(-80,-58,Math.toRadians(90)),Math.toRadians(180))
                .build();
        Action pixeldreapta=drive.actionBuilder(beginPose)
                .splineToLinearHeading(new Pose2d(-41, 35, Math.toRadians(236)), Math.toRadians(236))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-34, 55, Math.toRadians(270.00)), Math.toRadians(270.00))
                .setReversed(false)
                .build();

        Action pixelstanga=drive.actionBuilder(beginPose)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-33,35,Math.toRadians(325)),Math.toRadians(90))
                .setTangent(Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(-41,55,Math.toRadians(270)),Math.toRadians(-270))
                .build();

        Action pixelmijloc=drive.actionBuilder(beginPose)
                .splineToLinearHeading(new Pose2d(-36,32,Math.toRadians(270)),Math.toRadians(270))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-36, 55, Math.toRadians(270.00)), Math.toRadians(270.00))
                .setTangent(Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(-55,9,Math.toRadians(270)),Math.toRadians(270))
                .build();

        cPose=new Pose2d(-55,9,Math.toRadians(270));
        Action catrebackdropdreapta=drive.actionBuilder(cPose)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(22, 12, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(51, 37.5, Math.toRadians(0)), Math.toRadians(0))
                .build();
        cPose= new Pose2d(51,37.5,Math.toRadians(0));
        Action catrestackmiddle=drive.actionBuilder(cPose)
                //.setTangent(Math.toRadians(-140))
                //.splineToLinearHeading(new Pose2d(-55,4,Math.toRadians(0)),Math.toRadians(-180))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(9,0,Math.toRadians(0)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-61,7.5,Math.toRadians(0)),Math.toRadians(180))
                .afterDisp(1,new SequentialAction(
                        new ParallelAction(
                                intake.reversePixel(),
                                intake.angle(5)
                        ),
                        //new SleepAction(1.5),
                        new ParallelAction(
                                outtake.bottomHook("closed"),outtake.upperHook("closed")
                        ),
                        intake.stop()
                ))
                .build();
        cPose= new Pose2d(-61,7.5,Math.toRadians(0));
        Action catrebackdropmiddle=drive.actionBuilder(cPose)
                .setReversed(false)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-58, 8, Math.toRadians(0)), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(20, 8, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(52,34,Math.toRadians(0)),Math.toRadians(0))
                .build();
        cPose= new Pose2d(-34,55,Math.toRadians(270));
        Action catrebackdrop=drive.actionBuilder(cPose)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-35, 10, Math.toRadians(270)), Math.toRadians(270.00))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(22, 12, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(51, 31, Math.toRadians(0)), Math.toRadians(0))
                .build();
        cPose= new Pose2d(-40,55,Math.toRadians(270));
        Action catrebackdropleft=drive.actionBuilder(cPose)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-34, 9, Math.toRadians(270)), Math.toRadians(270.00))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(22, 9, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(50, 41, Math.toRadians(0)), Math.toRadians(0))
                .build();
        cPose= new Pose2d(51,44,Math.toRadians(0));
        Action catrestackleft=drive.actionBuilder(cPose)
                //.setTangent(Math.toRadians(-140))
                //.splineToLinearHeading(new Pose2d(-55,4,Math.toRadians(0)),Math.toRadians(-180))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(9,0,Math.toRadians(0)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-63,6,Math.toRadians(0)),Math.toRadians(180))
                .afterDisp(1,new SequentialAction(
                        new ParallelAction(
                                intake.reversePixel(),
                                intake.angle(5)
                        ),
                        //new SleepAction(1.5),
                        new ParallelAction(
                                outtake.bottomHook("closed"),outtake.upperHook("closed")
                        ),
                        intake.stop()
                ))
                .build();
        cPose= new Pose2d(-61,7.5,Math.toRadians(0));
        Action catrebackdrop2left=drive.actionBuilder(cPose)
                .setReversed(false)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-58, 10, Math.toRadians(0)), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(20, 10, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(50,34,Math.toRadians(0)),Math.toRadians(0))
                .build();
        cPose= new Pose2d(51,33,Math.toRadians(0));
        Action catrestack=drive.actionBuilder(cPose)
                //.setTangent(Math.toRadians(-140))
                //.splineToLinearHeading(new Pose2d(-55,4,Math.toRadians(0)),Math.toRadians(-180))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(9,3,Math.toRadians(0)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-60.5,9,Math.toRadians(0)),Math.toRadians(180))
                .afterDisp(1,new SequentialAction(
                        new ParallelAction(
                                intake.reversePixel(),
                                intake.angle(5)
                        ),
                        //new SleepAction(1.5),
                        new ParallelAction(
                                outtake.bottomHook("closed"),outtake.upperHook("closed")
                        ),
                        intake.stop()
                ))
                .build();
        cPose= new Pose2d(-60.5,9,Math.toRadians(0));
        Action catrebackdrop2=drive.actionBuilder(cPose)
                .setReversed(false)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-58, 7, Math.toRadians(0)), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(20, 7, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(51.3,34,Math.toRadians(0)),Math.toRadians(0))
                .build();
        //.setTangent(0)
        //.splineToLinearHeading(new Pose2d(8, 4, Math.toRadians(0)), Math.toRadians(0))
        cPose=new Pose2d(50,34,Math.toRadians(0));
        Action Parking=drive.actionBuilder(cPose)
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(49,20,Math.toRadians(0)),Math.toRadians(90))
                .build();
        waitForStart();

        externalCamera.stopStreaming();
        externalCamera.closeCameraDevice();

        if(PropZone=="LEFT"){
            Actions.runBlocking(new SequentialAction(
                    pixelstanga,
                    new SequentialAction(
                            new SequentialAction(
                                    catrebackdropleft,
                                    new ParallelAction(
                                            outtake.pivot(DefVal.pivot60),
                                            outtake.roll(DefVal.roll60),
                                            outtake.runToPosition("autonom")
                                    )
                            ),
                            new SleepAction(1),
                            new ParallelAction(
                                    outtake.upperHook("open"),
                                    outtake.bottomHook("open")
                            ),
                            new SleepAction(1),
                            new ParallelAction(
                                    outtake.pivot(DefVal.pivot0),
                                    outtake.roll(DefVal.roll0)
                            ),
                            new SleepAction(1),
                            outtake.runToPosition("ground")
                    ),
                    catrestackleft,
                    new SequentialAction(
                            new SequentialAction(
                                    catrebackdrop2left,
                                    new ParallelAction(
                                            outtake.pivot(DefVal.pivot60),
                                            outtake.roll(DefVal.roll60),
                                            outtake.runToPosition("low")
                                    )
                            ),
                            new SleepAction(1),
                            new ParallelAction(
                                    outtake.upperHook("open"),
                                    outtake.bottomHook("open")
                            ),
                            new SleepAction(1),
                            new ParallelAction(
                                    outtake.pivot(DefVal.pivot0),
                                    outtake.roll(DefVal.roll0)
                            ),
                            new SleepAction(1),
                            outtake.runToPosition("ground")
                    ),
                    Parking
            ));
        } else if(PropZone=="MIDDLE") {
            Actions.runBlocking(new SequentialAction(
                    pixelmijloc,
                    new SequentialAction(
                            new SequentialAction(
                                    catrebackdropdreapta,
                                    new ParallelAction(
                                            outtake.pivot(DefVal.pivot60),
                                            outtake.roll(DefVal.roll60),
                                            outtake.runToPosition("autonom")
                                    )
                            ),
                            new SleepAction(1),
                            new ParallelAction(
                                    outtake.upperHook("open"),
                                    outtake.bottomHook("open")
                            ),
                            new SleepAction(1),
                            new ParallelAction(
                                    outtake.pivot(DefVal.pivot0),
                                    outtake.roll(DefVal.roll0)
                            ),
                            new SleepAction(1),
                            outtake.runToPosition("ground")
                    ),
                    catrestack,
                    new SequentialAction(
                            new SequentialAction(
                                    catrebackdropmiddle,
                                    new ParallelAction(
                                            outtake.pivot(DefVal.pivot60),
                                            outtake.roll(DefVal.roll60),
                                            outtake.runToPosition("low")
                                    )
                            ),
                            new SleepAction(1),
                            new ParallelAction(
                                    outtake.upperHook("open"),
                                    outtake.bottomHook("open")
                            ),
                            new SleepAction(1),
                            new ParallelAction(
                                    outtake.pivot(DefVal.pivot0),
                                    outtake.roll(DefVal.roll0)
                            ),
                            new SleepAction(1),
                            outtake.runToPosition("ground")
                    ),
                    Parking
            ));
        } else if(PropZone=="RIGHT") {
            Actions.runBlocking(new SequentialAction(
                    pixeldreapta,
                    new SequentialAction(
                            new SequentialAction(
                                    catrebackdrop,
                                    new ParallelAction(
                                            outtake.pivot(DefVal.pivot60),
                                            outtake.roll(DefVal.roll60),
                                            outtake.runToPosition("autonom")
                                    )
                            ),
                            new SleepAction(1),
                            new ParallelAction(
                                    outtake.upperHook("open"),
                                    outtake.bottomHook("open")
                            ),
                            new SleepAction(1),
                            new ParallelAction(
                                    outtake.pivot(DefVal.pivot0),
                                    outtake.roll(DefVal.roll0)
                            ),
                            new SleepAction(1),
                            outtake.runToPosition("ground")
                    ),
                    catrestack,
                    new SequentialAction(
                            new SequentialAction(
                                    catrebackdrop2,
                                    new ParallelAction(
                                            outtake.pivot(DefVal.pivot60),
                                            outtake.roll(DefVal.roll60),
                                            outtake.runToPosition("low")
                                    )
                            ),
                            new SleepAction(1),
                            new ParallelAction(
                                    outtake.upperHook("open"),
                                    outtake.bottomHook("open")
                            ),
                            new SleepAction(1),
                            new ParallelAction(
                                    outtake.pivot(DefVal.pivot0),
                                    outtake.roll(DefVal.roll0)
                            ),
                            new SleepAction(1),
                            outtake.runToPosition("ground")
                    ),
                    Parking
            ));
        }
        while (opModeIsActive()) {
            //telemetry
        }



        while (opModeIsActive()) {

        }
    }









    private void initExternalCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );

        externalCamera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "WebcamG"), cameraMonitorViewId
        );

        pipeline = new nume();
        externalCamera.setPipeline(pipeline);

        /// externalCamera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        externalCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                externalCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public class nume extends OpenCvPipeline {
        public Rect rect1 = new Rect(0, 116, 75, 76);
        public Rect rect2 = new Rect(160, 102, 75, 76);

        private Mat hsvImage = new Mat();
        private Mat mask = new Mat();
        private Mat hierarchy = new Mat();

        private List<MatOfPoint> contours = new ArrayList<>();
        private Point squareCenter = new Point();
        public Scalar nonSelectedColor = new Scalar(255, 0, 0);
        public Scalar selectedColor = new Scalar(0, 0, 255);
        // public Scalar lowerBlue = new Scalar(106, 165, 55);
        // public Scalar upperBlue = new Scalar(230, 255, 255);
        public Scalar lowerBlue = new Scalar(106, 100, 50);
        public Scalar upperBlue = new Scalar(230, 255, 255);
        private int selectedRect = -1;

        public String isPointInsideRect() {
            if (rect1.contains(squareCenter)) {
                return "LEFT";
            } else if (rect2.contains(squareCenter)) {
                return "MIDDLE";
            } else {
                return "RIGHT";
            }
        }


        @Override
        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);

            Core.inRange(hsvImage, lowerBlue, upperBlue, mask);

            contours.clear();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            Mat processedFrame = input.clone();
            Imgproc.drawContours(processedFrame, contours, -1, new Scalar(0, 255, 0), 2);

            calculateSquarePosition();

            String zone = isPointInsideRect();

            if (!contours.isEmpty()) {
                String coordinates = "X: " + String.format("%.2f", squareCenter.x) + " Y: " + String.format("%.2f", squareCenter.y);
                Imgproc.putText(
                        processedFrame,
                        coordinates,
                        new Point(10, 130),
                        Imgproc.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        new Scalar(0, 0, 255),
                        1
                );
                Imgproc.putText(
                        processedFrame,
                        "Zone: " + zone,
                        new Point(10, 100),
                        Imgproc.FONT_HERSHEY_SIMPLEX,
                        1,
                        new Scalar(255, 255, 255),
                        2
                );
            }
//            Imgproc.putText(
//                    processedFrame,
//                    "Area",
//                    new Point(30, 150),
//                    Imgproc.FONT_HERSHEY_SIMPLEX,
//                    0.5,
//                    new Scalar(0, 0, 255),
//                    1
//            );

            drawRectangles(processedFrame);

            return processedFrame;
        }

        public Point getSquareCenter() {
            return squareCenter;
        }


        private void drawRectangles(Mat input) {
            Imgproc.rectangle(input, rect1, nonSelectedColor);
            Imgproc.rectangle(input, rect2, nonSelectedColor);


            switch (selectedRect) {
                case 1:
                    Imgproc.rectangle(input, rect1, selectedColor);
                    break;
                case 2:
                    Imgproc.rectangle(input, rect2, selectedColor);
                    break;
            }
        }

        //        private void calculateSquarePosition() {
//            squareCenter.x = 0;
//            squareCenter.y = 0;
//
//            if (!contours.isEmpty()) {
//                double maxArea = -1;
//                int maxAreaIdx = -1;
//                for (int i = 0; i < contours.size(); i++) {
//                    double area = Imgproc.contourArea(contours.get(i));
//                    if (area > maxArea) {
//                        maxArea = area;
//                        maxAreaIdx = i;
//                    }
//
//                }
//
//                if (maxAreaIdx != -1) {
//                    // Calculate the center of the largest contour
//                    Moments moments = Imgproc.moments(contours.get(maxAreaIdx));
//                    squareCenter.x = moments.m10 / moments.m00;
//                    squareCenter.y = moments.m01 / moments.m00;
//                }
//            }
//        }
        private void calculateSquarePosition() {
            squareCenter.x = 0;
            squareCenter.y = 0;

            if (!contours.isEmpty()) {
                double maxArea = -1;
                int maxAreaIdx = -1;
                double minContourArea =  190;

                for (int i = 0; i < contours.size(); i++) {
                    double area = Imgproc.contourArea(contours.get(i));
                    if (area >= minContourArea && area > maxArea) {
                        maxArea = area;
                        maxAreaIdx = i;
                    }
                }

                if (maxAreaIdx != -1) {
                    Moments moments = Imgproc.moments(contours.get(maxAreaIdx));
                    squareCenter.x = moments.m10 / moments.m00;
                    squareCenter.y = moments.m01 / moments.m00;
                }
            }
        }

    }
}