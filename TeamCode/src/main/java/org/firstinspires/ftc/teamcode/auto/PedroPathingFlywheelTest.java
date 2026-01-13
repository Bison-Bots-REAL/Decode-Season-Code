package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


public class PedroPathingFlywheelTest {
    private DcMotorEx intake;

    private DcMotorEx shooter;

    private DcMotorEx pusherupper;

    private Servo ramp;

    private ElapsedTime stateTimer = new ElapsedTime();

    private enum FlywheelState {
        IDLE,
        SPIN,
        LAUNCH,
        SPIN2,
        PUSHER,
        LAUNCH2,
        SPIN3,
        PUSHER2,
        RESET
    }
    private FlywheelState flywheelState;

    // ------------------------- RAMP LOGIC------------------------------
    private double rampUpPosition = 0.12;
    private double rampDownPosition = 0.08;

    private double rampUpTime = 1;
    private double rampDownTime = 1;

    // --------------------FLYWHEEL CONSTANTS --------------------------------
    private int shotsRemain = 0;

    private double flywheelVelocity = 0;
    private double minFlywheelRPM = 1100;

    private double targetFlywheelRPM = 1131;

    private double flywheelMaxRevTime = 2;

    //FAR VALUES AREN'T TESTED YET
    private double minFarFlywheelRPM = 2000;

    private double targetFarFlywheelRPM = 2100;

    //-----------------------INTAKE CONSTANTS------------------------------
    private double intakePower = 0.6;

    //---------------------------PUSHERUPPER CONSTANTS-------------------------
    private double pusherPower = 0.8;

    private double pusherTimer = 1;


    public void init(HardwareMap hwMap){
         intake = hwMap.get(DcMotorEx.class, "intake"); // control 3
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

         shooter = hwMap.get(DcMotorEx.class, "launch"); // control 1
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(50, 0 ,0, 5);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        //TODO add PIDF tuning


         pusherupper = hwMap.get(DcMotorEx.class, "pusherupper"); // expand 2
        pusherupper.setDirection(DcMotorEx.Direction.FORWARD);
        pusherupper.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        ramp = hwMap.get(Servo.class, "ramp"); // control 0
        ramp.setDirection(Servo.Direction.FORWARD);

        flywheelState = FlywheelState.IDLE;

        shooter.setVelocity(0);
        ramp.setPosition(rampDownPosition);
    }

    public void update(){
        switch(flywheelState){
            case IDLE:
                if (shotsRemain > 0){
                    ramp.setPosition(rampDownPosition);
                    shooter.setVelocity(targetFlywheelRPM);
                    intake.setPower(intakePower);

                    stateTimer.reset();
                    flywheelState = FlywheelState.SPIN;
                }
                break;
            case SPIN:
                if (flywheelVelocity > minFlywheelRPM || stateTimer.seconds() > flywheelMaxRevTime){
                    ramp.setPosition(rampUpPosition);

                    stateTimer.reset();
                    flywheelState = FlywheelState.LAUNCH;
                }
                break;
            case LAUNCH:
                if (stateTimer.seconds() > rampUpTime){
                    ramp.setPosition(rampDownPosition);

                    stateTimer.reset();
                    flywheelState = FlywheelState.SPIN2;

                }
                break;
            case SPIN2:
                if (stateTimer.seconds() > rampDownTime){
                    pusherupper.setPower(pusherPower);

                    stateTimer.reset();
                    flywheelState = FlywheelState.PUSHER;
                }
                break;
            case PUSHER:
                if (stateTimer.seconds() > pusherTimer){
                    ramp.setPosition(rampUpPosition);
                    pusherupper.setPower(0);

                    stateTimer.reset();
                    flywheelState = FlywheelState.LAUNCH2;
                }
                break;
            case LAUNCH2:
                if (stateTimer.seconds() > rampUpTime){
                    ramp.setPosition(rampDownPosition);

                    stateTimer.reset();
                    flywheelState = FlywheelState.SPIN3;
                }
                break;
            case SPIN3:
                if(stateTimer.seconds() > rampDownTime){
                    pusherupper.setPower(pusherPower);

                    stateTimer.reset();
                    flywheelState = FlywheelState.PUSHER2;
                }
                break;
            case PUSHER2:
                if (stateTimer.seconds() > pusherTimer){
                    ramp.setPosition(rampUpPosition);
                    shotsRemain--;


                    stateTimer.reset();
                    flywheelState = FlywheelState.RESET;
                }
            case RESET:
                if (stateTimer.seconds() > rampUpTime){
                    ramp.setPosition(rampDownPosition);
                    pusherupper.setPower(0);



                    stateTimer.reset();
                    flywheelState = FlywheelState.IDLE;
                }
                break;

        }
    }
    public void fireShots(int numberOfShots){
        if (flywheelState == FlywheelState.IDLE){
            shotsRemain = numberOfShots;
        }
    }

    public boolean isBusy(){
        return flywheelState != FlywheelState.IDLE;
    }
}
