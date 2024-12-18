package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "Autonom", group = "Robot")
public class Autonom extends LinearOpMode {
    public DcMotor  fata_stanga   = null; //the left drivetrain motor
    public DcMotor  fata_dreapta  = null; //the right drivetrain motor
    public DcMotor  spate_stanga    = null;
    public DcMotor  spate_dreapta   = null;
    public DcMotor  motor_stanga         = null; //the arm motor
    public DcMotor  motor_glisiere        = null; //
    public Servo  servoRotire           = null; //the active servoRotire servo
    public Servo    cleste            = null; //the cleste servo


    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 99.5 // This is the exact gear ratio of the gobilda 60rpm motor
                    * 10 // This is the external gear reduction
                    * 1/360.0; // we want ticks per degree, not per rotation


    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double cosSusBrat = 100 * ARM_TICKS_PER_DEGREE;
    final double cosJosBrat = 90 * ARM_TICKS_PER_DEGREE;
    final double SpecimenBrat = 90 * ARM_TICKS_PER_DEGREE;
    final double servoRetras = 0.4;
    final double servoTras = 0.7;
    final double cleste_score = 0.5;

    /* Variables to store the positions that the cleste should be set to when folding in, or folding out. */
    final double cleste_deschis   = 1;
    final double cleste_inchis  = 0;


    /* A number in degrees that the triggers can adjust the arm position by */


    /* Variables that are used to set the arm to a specific position */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;


    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;


    // double rotirePosition = (int)servoTras;


    final double LIFT_TICKS_PER_MM = 384.5 / 120.0; // Encoder ticks per mm for your specific motor and pulley setup

    final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_LOW_BASKET = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_HIGH_BASKET = 348 * LIFT_TICKS_PER_MM;
    double liftPosition = LIFT_COLLAPSED;

    final double LIFT_MAX_POSITION = (int) (348 * LIFT_TICKS_PER_MM); // Fully extended for 240mm slide


    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;

    double armLiftComp = 0;


    @Override
    public void runOpMode() {

        /* Define and Initialize Motors */
        fata_stanga   = hardwareMap.get(DcMotor.class, "fata_stanga"); //the arm motor
        spate_stanga   = hardwareMap.get(DcMotor.class, "spate_stanga"); //the arm motor
        fata_dreapta   = hardwareMap.get(DcMotor.class, "fata_dreapta"); //the arm motor
        spate_dreapta   = hardwareMap.get(DcMotor.class, "spate_dreapta"); //the arm motor
        motor_glisiere = hardwareMap.dcMotor.get("motor_glisiere");
        motor_stanga   = hardwareMap.get(DcMotor.class, "motor_stanga"); //the arm motor


       /*
       we need to reverse the left side of the drivetrain so it doesn't turn when we ask all the
       drive motors to go forward.
        */

        fata_stanga.setDirection(DcMotor.Direction.REVERSE);
        spate_stanga.setDirection(DcMotor.Direction.REVERSE);

        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        fata_stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fata_dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spate_stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spate_dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_glisiere.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior. BRAKE);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) motor_stanga).setCurrentAlert(5, CurrentUnit.AMPS);

        /* Before starting the motor_stanga. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        motor_stanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_stanga.setTargetPosition(0);
        motor_stanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_stanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor_glisiere.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_glisiere.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_glisiere.setTargetPosition(0);
        motor_glisiere.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_glisiere.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Define and initialize servos.*/
        servoRotire = hardwareMap.get(Servo.class, "servoRotire");
        cleste  = hardwareMap.get(Servo.class, "cleste");

        /* Make sure that the servoRotire is off, and the cleste is folded in. */
        //  servoRotire.setPower(rotirePosition);


        cleste.setPosition(cleste_inchis);


        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        waitForStart();

        /* Run until the driver presses stop */
        while (opModeIsActive())

        {






            /* send telemetry to the driver of the arm's current position and target position */
            telemetry.addData("arm Target Position: ", motor_stanga.getTargetPosition());
            telemetry.addData("arm Encoder: ", motor_stanga.getCurrentPosition());
            telemetry.addData("lift variable", liftPosition);
            telemetry.addData("Lift Target Position",motor_glisiere.getTargetPosition());
            telemetry.addData("lift current position", motor_glisiere.getCurrentPosition());
            telemetry.addData("motor_glisiere Current:",((DcMotorEx) motor_glisiere).getCurrent(CurrentUnit.AMPS));
            telemetry.update();



        }
    }
}