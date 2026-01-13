package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// This TeleOp mode is for Everything
@Autonomous

public class BasicAuto extends LinearOpMode{

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "rightBack");


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        double frontLeftPower  = 0.5;
        double frontRightPower = 0.5;
        double backLeftPower   = 0.5;
        double backRightPower  = 0.5;

        // Send calculated power to wheels
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
        sleep(1250);
    }
}
