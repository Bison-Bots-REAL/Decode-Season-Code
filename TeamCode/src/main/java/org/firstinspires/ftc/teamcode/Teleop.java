package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Teleop extends LinearOpMode{

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initializing the motor direction and names
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");

        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotor launch = hardwareMap.get(DcMotor.class, "launch");

        Servo ramp = hardwareMap.get(Servo.class, "ramp");

        ramp.setDirection(Servo.Direction.FORWARD);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(DcMotor.Direction.REVERSE);
        launch.setDirection(DcMotor.Direction.FORWARD);

        double rampPosition = 0.2;
        ramp.setPosition(rampPosition);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        double lowPower = 0.75;

        double lowIntake = 1.0;
        double lowLaunch = 1.0;

        // run until the end of the match (driver presses STOP)

        double frontLeftPower = 0;
        double backLeftPower = 0;

        double frontRightPower = 0;
        double backRightPower = 0;

        double LauncherPower = 0;
        double IntakePower = 0;

        while (opModeIsActive()) {
            double max;

            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            frontLeftPower = axial + lateral + yaw;
            frontRightPower = axial - lateral - yaw;
            backLeftPower = axial - lateral + yaw;
            backRightPower = axial + lateral - yaw;

            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            IntakePower *= lowIntake;

            LauncherPower *= lowLaunch;

            frontLeftPower = frontLeftPower * lowPower;
            frontRightPower = frontRightPower * lowPower;
            backLeftPower = backLeftPower * lowPower;
            backRightPower = backRightPower * lowPower;

            if (gamepad2.y)
            {
                rampPosition = 0.3;
            }
            else if(gamepad2.a)
            {
                rampPosition = 0;
            }

            if (gamepad2.left_bumper)
            {
                IntakePower = 1.0;
            }
            else if (gamepad2.right_bumper)
            {
                LauncherPower = 0.90;
            }
            else if (gamepad2.b)
            {
                IntakePower = 0;
                LauncherPower = 0;
            }

            // Send calculated power to the motor and servos
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            launch.setPower(LauncherPower);
            intake.setPower(IntakePower);

            ramp.setPosition(rampPosition);
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
        telemetry.addData("Launcher", "%4.2f", LauncherPower);
        telemetry.addData("Intake", "%4.2f", IntakePower);
        telemetry.update();
    }
}