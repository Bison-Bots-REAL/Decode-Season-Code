package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous
public class NearBluepreloadonly extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(-55, -46, Math.toRadians(55));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        /// Launcher
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotor launch = hardwareMap.get(DcMotor.class, "launch");
        launch.setDirection(DcMotor.Direction.REVERSE);
        launch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotor pusherupper = hardwareMap.get(DcMotor.class, "pusherupper");
        pusherupper.setDirection(DcMotorSimple.Direction.REVERSE);
        pusherupper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo ramp = hardwareMap.get(Servo.class, "ramp");
        ramp.setDirection(Servo.Direction.FORWARD);

        ramp.setPosition(0);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .stopAndAdd(telemetryPacket -> {
                            intake.setPower(1);
                            launch.setPower(0.6);
                            return false;
                        })

                        //.strafeTo(new Vector2d(50,15))
                        //.turn(Math.toRadians(-20))
                        .splineTo(new Vector2d(-18,-20),Math.toRadians(220))
                        .waitSeconds(0.2)

                        .stopAndAdd(telemetryPacket -> {
                            ramp.setPosition(0.085);
                            return false;
                        })

                        .waitSeconds(2)

                        .stopAndAdd(telemetryPacket -> {
                            ramp.setPosition(0);
                            return false;
                        })

                        .waitSeconds(2)

                        .stopAndAdd(telemetryPacket -> {
                            pusherupper.setPower(0.8);
                            return false;
                        })

                        .waitSeconds(0.8)

                        .stopAndAdd(telemetryPacket -> {
                            pusherupper.setPower(0);
                            ramp.setPosition(0.085);
                            return false;
                        })

                        .waitSeconds(2)

                        .stopAndAdd(telemetryPacket -> {
                            pusherupper.setPower(0.8);
                            return false;
                        })

                        .waitSeconds(1.8)

                        .stopAndAdd(telemetryPacket -> {
                            launch.setPower(0);
                            pusherupper.setPower(0);
                            ramp.setPosition(0);
                            return false;
                        })

                        /*
                        //.turnTo(Math.toRadians(90))
                        .splineTo(new Vector2d(-11,-28),Math.toRadians(-90))
                        //.strafeTo(new Vector2d(-11,28))
                        .strafeTo(new Vector2d(-11,-42))

                        .stopAndAdd(telemetryPacket -> {
                            launch.setPower(0.6);
                            return false;
                        })

                        //.turnTo(Math.toRadians(160))
                        .splineTo(new Vector2d(-18,-20),Math.toRadians(220))
                        .waitSeconds(0.2)

                        .stopAndAdd(telemetryPacket -> {
                            ramp.setPosition(0.085);
                            return false;
                        })

                        .waitSeconds(0.1)

                        .stopAndAdd(telemetryPacket -> {
                            ramp.setPosition(0);
                            pusherupper.setPower(0.8);
                            return false;
                        })

                        .waitSeconds(2.1)

                        .stopAndAdd(telemetryPacket -> {
                            pusherupper.setPower(0.5);
                            ramp.setPosition(0.085);
                            return false;
                        })

                        .waitSeconds(1.8)

                        .stopAndAdd(telemetryPacket -> {
                            launch.setPower(0);
                            pusherupper.setPower(0);
                            return false;
                        })

                        //.turnTo(Math.toRadians(90))
                        .splineTo(new Vector2d(13,-28),Math.toRadians(-90))
                        //.strafeTo(new Vector2d(13,28))
                        .strafeTo(new Vector2d(13,-42))

                        .stopAndAdd(telemetryPacket -> {
                            launch.setPower(0.6);
                            return false;
                        })

                        //.turnTo(Math.toRadians(160))
                        .splineTo(new Vector2d(-18,-20),Math.toRadians(220))
                        .waitSeconds(0.2)

                        .stopAndAdd(telemetryPacket -> {
                            ramp.setPosition(0.085);
                            return false;
                        })

                        .waitSeconds(0.1)

                        .stopAndAdd(telemetryPacket -> {
                            ramp.setPosition(0);
                            pusherupper.setPower(0.8);
                            return false;
                        })

                        .waitSeconds(2.1)

                        .stopAndAdd(telemetryPacket -> {
                            pusherupper.setPower(0.5);
                            ramp.setPosition(0.085);
                            return false;
                        })

                        .waitSeconds(1.8)

                        .stopAndAdd(telemetryPacket -> {
                            launch.setPower(0);
                            pusherupper.setPower(0);
                            return false;
                        })

                        /*
                        //.turnTo(Math.toRadians(90))
                        .splineTo(new Vector2d(36,-28),Math.toRadians(-90))
                        //.strafeTo(new Vector2d(37,28))
                        .strafeTo(new Vector2d(36,-46))

                        //.turnTo(Math.toRadians(160))
                        .splineTo(new Vector2d(-18,-20),Math.toRadians(220))
                        .waitSeconds(0.5)
                         */

                        .splineTo(new Vector2d(6,-15),Math.toRadians(0))
                        //.turnTo(Math.toRadians(0))
                        .build());
    }
}
