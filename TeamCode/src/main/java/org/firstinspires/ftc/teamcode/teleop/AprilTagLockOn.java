package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class AprilTagLockOn extends OpMode {
    private final AprilTagWebcam cam = new AprilTagWebcam();
    private final MecanumDrive drive = new MecanumDrive();

    //--------------- PD controller -------------------------
    double kp = 0.090;
    double error = 0;
    double lastError = 0;
    double goalX = 0;
    double angleTolerance = 0.2;
    double kD = 0.0006;
    double curTime = 0;
    double lastTime = 0;

    // --------------driving setup ----------------
    double forward, strafe, rotate;

    // ---------------- controller based PD tuning -------------
    double[] stepSizes = {0.1, 0.001, 0.0001};
    int stepIndex = 1;

    @Override
    public void init() {
        cam.init(hardwareMap, telemetry);
        drive.init(hardwareMap);

        telemetry.addLine("initialized");
    }

    @Override
    public void start() {
        resetRuntime();
        curTime = getRuntime();
    }

    @Override
    public void loop() {
        // Get Inputs
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        // Get April Tag Info
        cam.update();
        AprilTagDetection id20 = cam.getTagbyId(20);

        // auto align logic
        if (gamepad1.left_trigger > 0.2) {
            if (id20 != null) {
                error = goalX - id20.ftcPose.bearing;

                if (Math.abs(error) < angleTolerance) {
                    rotate = 0;
                } else {
                    double pTerm = error * kp;

                    curTime = getRuntime();
                    double dT = curTime - lastTime;
                    double dTerm = ((error - lastError) / dT) * kD;

                    rotate = Range.clip(pTerm + dTerm, -0.4, 0.4);

                    lastError = error;
                    lastTime = curTime;
                }
            }
        } else {
            lastError = 0;
            lastTime = getRuntime();
        }

        // drive
        drive.drive(-forward, strafe, rotate);

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            kp -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            kp += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            kD += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            kD -= stepSizes[stepIndex];
        }

        // Telemetry
        if (id20 != null) {
            if (gamepad1.left_trigger > 0.3) {
                telemetry.addLine("AUTO ALIGN");
            }
            cam.displayDetectionTelemetry(id20);
            telemetry.addData("Error", error);
        } else {
            telemetry.addLine("MANUAL Rotate Mode");
        }
        telemetry.addLine("---------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-PAD L/R)", kp);
        telemetry.addData("Tuning D", "%.4f (D-PAD U/D)", kD);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);

    }
}
