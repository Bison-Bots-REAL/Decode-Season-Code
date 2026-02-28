package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;

@TeleOp
public class CameraTest extends OpMode {

    AprilTagWebcam camera = new AprilTagWebcam();

    @Override
    public void init() {
        camera.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        camera.update();
    }
}
