package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.521)
            .forwardZeroPowerAcceleration(-45.78394454996832)
            .lateralZeroPowerAcceleration(-51.77983429663153)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.07,0,0.007, 0.04))
            .headingPIDFCoefficients(new PIDFCoefficients (0.88, 0, 0.04, 0.04))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.016, 0.0, 0.00015, 0.025, 0.025))
            .centripetalScaling(0.0005)

            ;


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(61.78941966506094)
            .yVelocity(47.4476029842621)



            ;



    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(0.003016643976216495)
            .strafeTicksToInches(0.003048767190142741)
            .turnTicksToInches(0.00289042958032866)
            .leftPodY(4.75)
            .rightPodY(-4.75)
            .strafePodX(-8)
            .leftEncoder_HardwareMapName("pusherupper")
            .rightEncoder_HardwareMapName("intake")
            .strafeEncoder_HardwareMapName("rightFront")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD);



    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 2.8, 0.7);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();
    }
}
