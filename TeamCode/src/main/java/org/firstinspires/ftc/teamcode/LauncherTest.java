package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class LauncherTest extends LinearOpMode{

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initializing the motor direction and names
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotor launch = hardwareMap.get(DcMotor.class, "launch");

        intake.setDirection(DcMotor.Direction.REVERSE);
        launch.setDirection(DcMotor.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        double lowIntake = 1.0;
        double lowLaunch = 1.0;

        // run until the end of the match (driver presses STOP)

        double LauncherPower = 0;
        double IntakePower = 0;

        while (opModeIsActive()) {

            IntakePower = gamepad1.right_trigger;

            IntakePower *= lowIntake;

            LauncherPower = gamepad1.left_trigger;

            LauncherPower *= lowLaunch;


            // Send calculated power to the motor and servos
            launch.setPower(LauncherPower);
            intake.setPower(IntakePower);
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.addData("Launcher", "%4.2f", LauncherPower);
        telemetry.addData("Launcher", "%4.2f", IntakePower);
        telemetry.update();
    }
}