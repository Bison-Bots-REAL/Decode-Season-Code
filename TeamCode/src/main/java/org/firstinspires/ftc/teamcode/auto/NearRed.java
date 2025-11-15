package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous
public class NearRed extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(-61, 10, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        //.strafeTo(new Vector2d(50,15))
                        //.turn(Math.toRadians(-20))
                        .splineTo(new Vector2d(-14,15),Math.toRadians(140))
                        .waitSeconds(0.5)

                        //.turnTo(Math.toRadians(90))
                        .splineTo(new Vector2d(-11,28),Math.toRadians(90))
                        //.strafeTo(new Vector2d(-11,28))
                        .strafeTo(new Vector2d(-11,42))

                        //.turnTo(Math.toRadians(160))
                        .splineTo(new Vector2d(-14,15),Math.toRadians(140))
                        .waitSeconds(0.5)

                        //.turnTo(Math.toRadians(90))
                        .splineTo(new Vector2d(13,28),Math.toRadians(90))
                        //.strafeTo(new Vector2d(13,28))
                        .strafeTo(new Vector2d(13,42))

                        //.turnTo(Math.toRadians(160))
                        .splineTo(new Vector2d(-14,15),Math.toRadians(140))
                        .waitSeconds(0.5)

                        //.turnTo(Math.toRadians(90))
                        .splineTo(new Vector2d(36,28),Math.toRadians(90))
                        //.strafeTo(new Vector2d(37,28))
                        .strafeTo(new Vector2d(36,46))

                        //.turnTo(Math.toRadians(160))
                        .splineTo(new Vector2d(-14,15),Math.toRadians(140))
                        .waitSeconds(0.5)

                        .splineTo(new Vector2d(6,15),Math.toRadians(0))
                        //.turnTo(Math.toRadians(0))
                        .build());
    }
}
