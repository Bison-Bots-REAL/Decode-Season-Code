package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Teleop", group = "Main")
public class TrackingTeleop extends OpMode {
    private final double intakeSpeed = 0.6;
    private final double fastDriveSpeed = 1.0;
    private final double driveSpeed = 0.75;
    private final double fastLaunchSpeed = 1550;
    private final double launchSpeed = 1200;
    private final double rampUpPosition = 0.08;
    private final double rampDownPosition = 0.04;

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

    /// Launcher Hardware
    DcMotor intake;
    DcMotorEx launch;
    DcMotorEx launch2;
    DcMotor pusherupper;
    Servo ramp;
    PIDFCoefficients pidfCoefficients;
    PIDFCoefficients pidfCoefficients2;

    private boolean fastlaunch = false;
    private boolean launching = false;

    private boolean reversedintake = false;
    private boolean intaking = false;

    private double driveLowPower = 1.0;
    private double intakeLowPower = 1.0;
    private double launchLowPower = 1.0;
    private double pusherupperLowPower = 0.6;

    private double frontLeftPower;
    private double backLeftPower;

    private double frontRightPower;
    private double backRightPower;

    private double LauncherPower = 0;
    private double IntakePower = 0;
    private double PusherUpperPower = 0;

    private double rampPosition = rampDownPosition;

    @Override
    public void init() {
        cam.init(hardwareMap, telemetry);
        drive.init(hardwareMap);

        /// Launcher
        intake = hardwareMap.get(DcMotor.class, "intake"); // control 3
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launch = hardwareMap.get(DcMotorEx.class, "launch"); // control 1
        launch.setDirection(DcMotorEx.Direction.REVERSE);
        launch.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        launch.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pidfCoefficients = new PIDFCoefficients(300, 0 ,0, 13);
        launch.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        launch2 = hardwareMap.get(DcMotorEx.class, "launch2"); //
        launch2.setDirection(DcMotorEx.Direction.FORWARD);
        launch2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        launch2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pidfCoefficients2 = new PIDFCoefficients(300, 0 ,0, 13);
        launch2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients2);

        pusherupper = hardwareMap.get(DcMotor.class, "pusherupper"); // expand 2
        pusherupper.setDirection(DcMotorSimple.Direction.FORWARD);
        pusherupper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ramp = hardwareMap.get(Servo.class, "ramp"); // control 0
        ramp.setDirection(Servo.Direction.FORWARD);

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

        /// Launcher Inputs
        if (gamepad2.yWasPressed()) {
            rampPosition = rampUpPosition;
        } else if (gamepad2.aWasPressed()) {
            rampPosition = rampDownPosition;
        }

        if (gamepad2.dpadDownWasPressed()) {
            fastlaunch = false;
        } else if (gamepad2.dpadUpWasPressed()) {
            fastlaunch = true;
        }

        if (launching) {
            if (gamepad1.backWasPressed()) {
                LauncherPower = -fastLaunchSpeed;
            } else if (fastlaunch) {
                LauncherPower = fastLaunchSpeed;
            } else {
                LauncherPower = launchSpeed;
            }
        }

        if (gamepad2.leftBumperWasPressed()) {
            intaking = true;
        }

        if (gamepad2.xWasPressed()) {
            reversedintake = true;
        } else if (gamepad2.xWasReleased()) {
            reversedintake = false;
        }

        if (gamepad2.rightBumperWasPressed()) {
            launching = true;
        } else if (gamepad2.bWasPressed()) {
            LauncherPower = 0;
            launching = false;
        }

        if (intaking) {
            if (reversedintake) {
                IntakePower = -intakeSpeed;
            } else {
                IntakePower = intakeSpeed;
            }
        }

        if (gamepad1.right_trigger > 0.05) {
            driveLowPower = fastDriveSpeed;
        } else {
            driveLowPower = driveSpeed;
        }

        if (gamepad2.backWasPressed()) {
            rampPosition = 0.08;
            IntakePower = 0;
            LauncherPower = 0;
            launching = false;
        }

        if (gamepad2.startWasPressed()) {
            IntakePower = 0;
        }

        PusherUpperPower = (gamepad2.right_trigger - gamepad2.left_trigger) * 0.8;

        // Apply Low Powers to components to slow down if needed
        IntakePower *= intakeLowPower;
        LauncherPower *= launchLowPower;
        PusherUpperPower *= pusherupperLowPower;

        frontLeftPower = frontLeftPower * driveLowPower;
        frontRightPower = frontRightPower * driveLowPower;
        backLeftPower = backLeftPower * driveLowPower;
        backRightPower = backRightPower * driveLowPower;

        launch.setVelocity(LauncherPower);
        launch2.setVelocity(LauncherPower);
        intake.setPower(IntakePower);
        pusherupper.setPower(PusherUpperPower);

        ramp.setPosition(rampPosition);

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
    }
}
