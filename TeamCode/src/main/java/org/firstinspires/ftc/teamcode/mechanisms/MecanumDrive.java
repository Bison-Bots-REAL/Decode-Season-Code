package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrive {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private IMU imu;

    public void init(HardwareMap hwmap) {
        frontLeftMotor = hwmap.get(DcMotor.class, "leftFront");
        backLeftMotor = hwmap.get(DcMotor.class, "leftBack");
        frontRightMotor = hwmap.get(DcMotor.class, "rightFront");
        backRightMotor = hwmap.get(DcMotor.class, "rightBack");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hwmap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public void drive(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeftMotor.setPower(maxSpeed * (frontLeftPower / maxPower));
        backLeftMotor.setPower(maxSpeed * (backLeftPower / maxPower));
        frontRightMotor.setPower(maxSpeed * (frontRightPower / maxPower));
        backRightMotor.setPower(maxSpeed * (backRightPower / maxPower));
    }

    public void driveFieldRelative(double forward, double strafe, double rotate) {
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
        );

        double nowForward = r * Math.sin(theta);
        double nowStrafe = r * Math.cos(theta);

        this.drive(nowForward, nowStrafe, rotate);
    }
}
