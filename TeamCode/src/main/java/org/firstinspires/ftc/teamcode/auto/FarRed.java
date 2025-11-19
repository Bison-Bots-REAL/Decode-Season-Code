package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Disabled
@Autonomous
public class FarRed extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(61, 10, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        //.strafeTo(new Vector2d(50,15))
                        //.turn(Math.toRadians(-20))
                        .splineTo(new Vector2d(50,15),Math.toRadians(200))
                        .waitSeconds(0.5)

                        //.turnTo(Math.toRadians(90))
                        .splineTo(new Vector2d(36,28),Math.toRadians(90))
                        //.strafeTo(new Vector2d(37,28))
                        .strafeTo(new Vector2d(36,50))

                        //.turnTo(Math.toRadians(160))
                        .splineTo(new Vector2d(50,15),Math.toRadians(200))
                        .waitSeconds(0.5)

                        //.turnTo(Math.toRadians(90))
                        .splineTo(new Vector2d(13,28),Math.toRadians(90))
                        //.strafeTo(new Vector2d(13,28))
                        .strafeTo(new Vector2d(13,50))

                        //.turnTo(Math.toRadians(160))
                        .splineTo(new Vector2d(50,15),Math.toRadians(200))
                        .waitSeconds(0.5)

                        //.turnTo(Math.toRadians(90))
                        .splineTo(new Vector2d(-11,28),Math.toRadians(90))
                        //.strafeTo(new Vector2d(-11,28))
                        .strafeTo(new Vector2d(-11,50))

                        .splineTo(new Vector2d(50,15),Math.toRadians(200))
                        .waitSeconds(0.5)

                        .strafeTo(new Vector2d(42,15))
                        .turnTo(Math.toRadians(180))
                        .build());
    }
}
