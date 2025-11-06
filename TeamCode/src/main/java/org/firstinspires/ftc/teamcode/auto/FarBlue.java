package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous
public class FarBlue extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(61, 10, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(56,15))
                        .turn(Math.toRadians(40))
                        .waitSeconds(0.5)

                        .turnTo(Math.toRadians(90))
                        .strafeTo(new Vector2d(36,28))
                        .strafeTo(new Vector2d(36,52))

                        .turnTo(Math.toRadians(220))
                        .splineToConstantHeading(new Vector2d(56,15),0)
                        .waitSeconds(0.5)

                        .turnTo(Math.toRadians(90))
                        .strafeTo(new Vector2d(12,28))
                        .strafeTo(new Vector2d(12,52))

                        .turnTo(Math.toRadians(220))
                        .splineToConstantHeading(new Vector2d(56,15),0)
                        .waitSeconds(0.5)

                        .turnTo(Math.toRadians(90))
                        .strafeTo(new Vector2d(-11,28))
                        .strafeTo(new Vector2d(-11,52))

                        .turnTo(Math.toRadians(220))
                        .splineToConstantHeading(new Vector2d(56,15),0)
                        .waitSeconds(0.5)

                        .turnTo(180)
                        .strafeTo(new Vector2d(46,15))
                        .build());
    }
}
