package org.firstinspires.ftc.teamcode.teleop.launchtesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class launcherencoderusingtest extends LinearOpMode {
    @Override
    public void runOpMode() {
        final double rampUpPosition = 0.08;
        final double rampDownPosition = 0;
        final double intakeSpeed = 0.6;
        double rampPosition = 0.0;

        boolean reversedintake = false;
        boolean intaking = false;

        DcMotor launcher = hardwareMap.get(DcMotor.class, "launch"); // Replace "launch" with your motor's name
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor intake = hardwareMap.get(DcMotor.class, "intake"); // control 3
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotor pusherupper = hardwareMap.get(DcMotor.class, "pusherupper"); // expand 2
        pusherupper.setDirection(DcMotorSimple.Direction.FORWARD);
        pusherupper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo ramp = hardwareMap.get(Servo.class, "ramp"); // control 0
        ramp.setDirection(Servo.Direction.FORWARD);

        // Set motor direction if needed
        // myMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Start the motor (adjust power as needed)
        double launcherPower = 0.0; // 1131 at 0.6
        launcher.setPower(launcherPower);

        double goalticks = 4.2;

        // Get initial encoder position
        double lastTicks = launcher.getCurrentPosition();

        // Get current encoder position
        double currentTicks;

        // Calculate ticks traveled
        double ticksPerSecond;

        int checks = 0;

        while (opModeIsActive()) {
            currentTicks = launcher.getCurrentPosition();
            ticksPerSecond = currentTicks - lastTicks;

            if (gamepad2.y) {
                rampPosition = rampUpPosition;
            } else if (gamepad2.a) {
                rampPosition = rampDownPosition;
            }

            if (gamepad2.left_bumper) {
                intaking = true;
            } else if (gamepad2.x) {
                reversedintake = true;
            } else if (!gamepad2.x) {
                reversedintake = false;
            }


            if (ticksPerSecond > goalticks) {
                launcherPower -= 0.001;

                if (launcherPower > 1) launcherPower = 1;

                launcher.setPower(launcherPower);
            } else if (ticksPerSecond < goalticks) {
                launcherPower += 0.001;

                if (launcherPower < 0) launcherPower = 0;

                launcher.setPower(launcherPower);
            }
            
            lastTicks = currentTicks;

            if (intaking) {
                if (reversedintake) {
                    intake.setPower(-intakeSpeed);
                } else {
                    intake.setPower(intakeSpeed);
                }
            }

            pusherupper.setPower((gamepad2.right_trigger - gamepad2.left_trigger) * 0.8);
            ramp.setPosition(rampPosition);

            checks += 1;

            telemetry.addData("Ticks per Second", ticksPerSecond);
            telemetry.addData("Current ticks", currentTicks);
            telemetry.addData("Last Ticks", lastTicks);
            telemetry.addData("Motor Power", launcherPower);
            telemetry.addData("Times Checked", checks);
            telemetry.update();
        }
    }
}
