package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous
public class PedroPathingCloseRed12ball extends OpMode{
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // -----------------------FLYWHEEL LOGIC---------------------------
    private PedroPathingFlywheelTest shooter = new PedroPathingFlywheelTest();

    private boolean shotsTriggered = false;

    public enum PathState{
        //START POSITION_END POSITION
        //DRIVE > MOVEMENT STATE
        //SHOOT > SCORING ARTIFACTS

        DRIVE_START_POS_SHOOT_POS,

        SHOOT_PRELOAD,

        DRIVE_PRELOAD_POS_MARK_ONE,

        SHOOT_MARK_ONE,

        DRIVE_PRELOAD_POS_MARK_TWO,

        SHOOT_MARK_TWO,

        DRIVE_PRELOAD_POS_MARK_THREE,

        SHOOT_MARK_THREE,

        LEAVE
    }

    // ----------------------End of Things---------------------------

    PathState pathState;

    private final Pose startPose = new Pose(122.828471411902,125.51691948658109,  Math.toRadians(45));

    private final Pose shootPose = new Pose (91.03033838973161,88.67794632438739, Math.toRadians(45));

    private final Pose markPose = new Pose (128.9031505250875,83.60793465577596, Math.toRadians(0));




    private PathChain driveStartPosShootPos, driveShootPosMarkPos, driveMarkPosShootPos, driveShootPosMark2Pos, driveMark2PosShootPos, driveShootPosMark3Pos, driveMark3PosShootPos, leave;

    public void buildPaths(){
        // put coordinates for starting position and end position
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
        driveShootPosMarkPos = follower.pathBuilder()
                .addPath(new BezierLine(
                        shootPose,
                        new Pose(markPose.getX() - 40, markPose.getY(), markPose.getHeading())
                ))
                .setLinearHeadingInterpolation(shootPose.getHeading(), markPose.getHeading())
                .addPath(new BezierLine(shootPose, markPose))
                .setConstantHeadingInterpolation(markPose.getHeading())
                .build();
        driveMarkPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(markPose, shootPose))
                .setLinearHeadingInterpolation(markPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosMark2Pos = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                shootPose,
                                new Pose(89.120, 56.942)
                        )
                )
                .setLinearHeadingInterpolation(shootPose.getHeading(), markPose.getHeading())
                .addPath(
                        new BezierLine(
                                new Pose(89.120, 56.942),
                                new Pose(134.518, 59.575)
                        )
                )
                .setConstantHeadingInterpolation(0)
                .build();
        driveMark2PosShootPos = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(129.518, 59.575),
                                new Pose(89.120, 56.942),
                                shootPose
                        )
                )
                .setLinearHeadingInterpolation(markPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosMark3Pos = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(84, 86.000),

                                new Pose(102, 36.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(102, 36.000),

                                new Pose(129, 36.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();
        driveMark3PosShootPos = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(15.000, 36.000),

                                new Pose(57.000, 86.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                .build();
        leave = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                shootPose,
                                new Pose(shootPose.getX() + 8, shootPose.getY() - 25)
                        )
                )
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch (pathState){
            case DRIVE_START_POS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD); //resets timer and makes new state
                break;
                //IMPORTANT FOR ADDING SHOT COMMAND TO OTHER THINGS USE AS REFERENCE
            case SHOOT_PRELOAD:
                if (!follower.isBusy()){
                    if(!shotsTriggered){
                        shooter.fireShots(1);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.isBusy()) {
                        //shots are done and free to transition
                        follower.followPath(driveShootPosMarkPos, true);
                        setPathState(PathState.DRIVE_PRELOAD_POS_MARK_ONE);
                        telemetry.addLine("Done Path 1");
                    }
                }
                break;
                // END OF SHOT COMMAND INTEGRATION
            case DRIVE_PRELOAD_POS_MARK_ONE:
                if (!follower.isBusy()){
                    //add transition state to next path
                    follower.followPath(driveMarkPosShootPos, true);
                    setPathState(PathState.SHOOT_MARK_ONE);
                    telemetry.addLine("Done Path 2");
                }
                break;
            case SHOOT_MARK_ONE:
                if (!follower.isBusy()){
                    if(!shotsTriggered){
                        shooter.fireShots(1);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.isBusy()) {
                        //shots are done and free to transition
                        follower.followPath(driveShootPosMark2Pos, true);
                        setPathState(PathState.DRIVE_PRELOAD_POS_MARK_TWO);
                        telemetry.addLine("Done Path 3");
                    }
                }
                break;
            case DRIVE_PRELOAD_POS_MARK_TWO:
                if (!follower.isBusy()){
                    //add transition state to next path
                    follower.followPath(driveMark2PosShootPos, true);
                    setPathState(PathState.SHOOT_MARK_TWO);
                    telemetry.addLine("Done Path 2");
                }
                break;
            case SHOOT_MARK_TWO:
                if (!follower.isBusy()){
                    if(!shotsTriggered){
                        shooter.fireShots(1);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.isBusy()) {
                        //shots are done and free to transition
                        //follower.followPath(driveShootPosMark2Pos, true);
                        //setPathState(PathState.DRIVE_PRELOAD_POS_MARK_ONE);
                        follower.followPath(driveShootPosMark3Pos, true);
                        setPathState(PathState.DRIVE_PRELOAD_POS_MARK_THREE);
                        telemetry.addLine("Done Auto");
                    }
                }
                break;
            case DRIVE_PRELOAD_POS_MARK_THREE:
                if (!follower.isBusy()){
                    //add transition state to next path
                    follower.followPath(driveMark3PosShootPos, true);
                    setPathState(PathState.SHOOT_MARK_THREE);
                    telemetry.addLine("Done Path 3");
                }
                break;
            case SHOOT_MARK_THREE:
                if (!follower.isBusy()){
                    if(!shotsTriggered){
                        shooter.fireShots(1);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.isBusy()) {
                        //shots are done and free to transition
                        follower.followPath(leave, true);
                        setPathState(PathState.LEAVE);
                        telemetry.addLine("Done Auto");
                    }
                }
                break;
            case LEAVE:
                if (!follower.isBusy()){
                    //add transition state to next path
                    telemetry.addLine("Done");
                }
                break;
            default:
                telemetry.addLine("No State Command");
                break;
        }

    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();

        shotsTriggered = false;
    }


    @Override
    public void init(){
        pathState = PathState.DRIVE_START_POS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        shooter.init(hardwareMap);

        buildPaths();
        follower.setPose(startPose); //Tells robot start position

    }


    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }



    @Override
    public void loop(){
        follower.update();
        shooter.update();
        statePathUpdate();


        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());

    }

}

