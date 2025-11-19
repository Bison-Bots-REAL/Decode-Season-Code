package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(group = "Main")
public class Teleop extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private int CameraStreamFrameRate = 12;

    @Override
    public void runOpMode() {
        boolean debug = gamepad1.guide;

        AprilTagProcessor apriltagprocessor = AprilTagProcessor.easyCreateWithDefaults();

        VisionPortal visionportal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(apriltagprocessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        if (debug) FtcDashboard.getInstance().startCameraStream(visionportal,CameraStreamFrameRate);

        // Initializing the motor direction and names
        /// Driving
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "leftFront"); // control 2
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        DcMotor backLeft = hardwareMap.get(DcMotor.class, "leftBack"); // expand 3
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        DcMotor frontRight = hardwareMap.get(DcMotor.class, "rightFront"); // control 0
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        DcMotor backRight = hardwareMap.get(DcMotor.class, "rightBack"); // expand 0
        backRight.setDirection(DcMotor.Direction.FORWARD);

        /// Launcher
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake"); // control 3
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotor launch = hardwareMap.get(DcMotor.class, "launch"); // control 1
        launch.setDirection(DcMotor.Direction.REVERSE);
        launch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotor pusherupper = hardwareMap.get(DcMotor.class, "pusherupper"); // expand 2
        pusherupper.setDirection(DcMotorSimple.Direction.REVERSE);
        pusherupper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo ramp = hardwareMap.get(Servo.class, "ramp"); // control 0
        ramp.setDirection(Servo.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Pauses the code here until the Play button is pressed after init
        waitForStart();
        runtime.reset();

        boolean fastlaunch = false;
        boolean launching = false;

        long lastTime = System.nanoTime();
        int framecount = 0;
        double currentFPS = 0.0;

        double driveLowPower = 0.75;
        double intakeLowPower = 1.0;
        double launchLowPower = 1.0;
        double pusherupperLowPower = 0.8;

        double frontLeftPower;
        double backLeftPower;

        double frontRightPower;
        double backRightPower;

        double LauncherPower = 0;
        double IntakePower = 0;
        double PusherUpperPower = 0;

        double rampPosition = 0.0;
        ramp.setPosition(rampPosition);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            /// Driving Input
            double max;

            // Get Movement Input
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Calculate Wheel Power from Input (I don't know how this was figured out don't change this)
            frontLeftPower = axial + lateral + yaw;
            frontRightPower = axial - lateral - yaw;
            backLeftPower = axial - lateral + yaw;
            backRightPower = axial + lateral - yaw;

            // Limit wheel power to the maximum it can handle
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            /// Launcher Inputs
            if (gamepad2.y) {
                rampPosition = 0.08;
            } else if (gamepad2.a) {
                rampPosition = 0;
            }

            if (gamepad2.dpad_down) {
                fastlaunch = false;
            } else if (gamepad2.dpad_up) {
                fastlaunch = true;
            }

            if (launching) {
                if (fastlaunch) {
                    LauncherPower = 1.0;
                } else {
                    LauncherPower = 0.6;
                }
            }

            if (gamepad2.left_bumper) {
                IntakePower = 0.6;
            } else if (gamepad2.x) {
                IntakePower = -0.6;
            } else if (gamepad2.right_bumper) {
                launching = true;
            } else if (gamepad2.b) {
                LauncherPower = 0;
                launching = false;
            }

            if (gamepad2.back) {
                rampPosition = 0;
                IntakePower = 0;
                PusherUpperPower = 0;
                LauncherPower = 0;
                launching = false;
            }

            if (gamepad2.start) {
                IntakePower = 0;
            }

            PusherUpperPower = gamepad2.right_trigger;

            // Apply Low Powers to components to slow down if needed
            IntakePower *= intakeLowPower;
            LauncherPower *= launchLowPower;
            PusherUpperPower *= pusherupperLowPower;

            frontLeftPower = frontLeftPower * driveLowPower;
            frontRightPower = frontRightPower * driveLowPower;
            backLeftPower = backLeftPower * driveLowPower;
            backRightPower = backRightPower * driveLowPower;

            // Send calculated power to the motors and servos
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            launch.setPower(LauncherPower);
            intake.setPower(IntakePower);
            pusherupper.setPower(PusherUpperPower);

            ramp.setPosition(rampPosition);

            /// Calculate Frame Rate (Only use in debugging keep commented out in competition)
            if (debug) {
                long currentTime = System.nanoTime();
                long elapsedTime = currentTime - lastTime;

                framecount++;

                if (elapsedTime >= 1000000000) {
                    currentFPS = (double) framecount / (elapsedTime / 1000000000.0);
                    framecount = 0;
                    lastTime = currentTime;
                }
            }

            /// Telemetry
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime);

            if (debug) {
                telemetry.addLine();
                telemetry.addLine("DEBUG");
                telemetry.addData("FPS", currentFPS);
            }

            telemetry.addLine();
            telemetry.addLine("LAUNCHER BOOLS");
            telemetry.addData("Launching?", launching);
            telemetry.addData("FastMode?", fastlaunch);

            telemetry.addLine();
            telemetry.addLine("MOTORS");
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.addData("Launcher", "%4.2f", LauncherPower);
            telemetry.addData("Intake", "%4.2f", IntakePower);
            telemetry.addData("PusherUpper", "%4.2f", PusherUpperPower);

            telemetry.addLine();
            telemetry.addLine("SERVOS");
            telemetry.addData("Ramp Target", "%4.2f", rampPosition);
            telemetry.addData("Ramp Position", "%4.2f", ramp.getPosition());

            telemetry.update();
            //if (debug) FtcDashboard.getInstance().sendTelemetryPacket(new TelemetryPacket()); // TODO fix this
        }
    }
}