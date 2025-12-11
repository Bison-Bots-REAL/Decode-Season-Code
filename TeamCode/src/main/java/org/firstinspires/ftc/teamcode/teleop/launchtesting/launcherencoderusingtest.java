package org.firstinspires.ftc.teamcode.teleop.launchtesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class launcherencoderusingtest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor launcher = hardwareMap.get(DcMotor.class, "launch"); // Replace "myMotor" with your motor's name
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motor direction if needed
        // myMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Start the motor (adjust power as needed)
        double launcherPower = 0.0; // 1131 at 0.6`j/
        launcher.setPower(launcherPower);

        int goalticks = 3;

        // Get initial encoder position
        int lastTicks = launcher.getCurrentPosition();

        // Get current encoder position
        int currentTicks;

        // Calculate ticks traveled
        int ticksPerSecond;

        while (opModeIsActive()) {
            currentTicks = launcher.getCurrentPosition();
            ticksPerSecond = currentTicks - lastTicks;

            if (ticksPerSecond > goalticks) {
                launcherPower -= 0.01;

                if (launcherPower > 1) launcherPower = 1;

                launcher.setPower(launcherPower);
            } else if (ticksPerSecond < goalticks) {
                launcherPower += 0.01;

                if (launcherPower < 0) launcherPower = 0;

                launcher.setPower(launcherPower);
            }
            
            lastTicks = currentTicks;

            telemetry.addData("Ticks per Second", ticksPerSecond);
            telemetry.addData("Current ticks", currentTicks);
            telemetry.addData("Last Ticks", lastTicks);
            telemetry.update();
        }
    }
}
