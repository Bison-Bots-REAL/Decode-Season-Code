package org.firstinspires.ftc.teamcode.teleop.launchtesting;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class launcherencodertest extends LinearOpMode{
    @Override
    public void runOpMode() {
        DcMotor launcher = hardwareMap.get(DcMotor.class, "launch"); // Replace "myMotor" with your motor's name
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        // Optional: Set motor direction if needed
        // myMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Start the motor (adjust power as needed)
        launcher.setPower(0.5);

        // Get initial encoder position
        int initialTicks = launcher.getCurrentPosition();

        // Wait for 1 second
        try {
            Thread.sleep(10000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        // Get final encoder position
        int finalTicks = launcher.getCurrentPosition();

        // Calculate ticks traveled
        int ticksPerSecond = finalTicks - initialTicks;

        // Stop the motor
        launcher.setPower(0);

        telemetry.addData("Ticks traveled per second", ticksPerSecond/10);
        telemetry.update();

        while (opModeIsActive()) {
            // Keep the Opmode active
        }
    }
}
