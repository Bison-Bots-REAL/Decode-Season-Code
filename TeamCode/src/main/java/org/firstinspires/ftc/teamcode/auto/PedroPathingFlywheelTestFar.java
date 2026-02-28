package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class PedroPathingFlywheelTestFar {
    private DcMotorEx intake;

    private DcMotorEx shooter;
    private DcMotorEx shooter2;

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
    private double rampUpPosition = 0.08;
    private double rampDownPosition = 0.04;

    private double rampUpTime = 0.6;
    private double rampDownTime = 0.75;

    // --------------------FLYWHEEL CONSTANTS --------------------------------
    private int shotsRemain = 0;

    private double flywheelVelocity = 0;
    private double minFlywheelRPM = 1200;

    private double targetFlywheelRPM = 1250;

    private double flywheelMaxRevTime = 1.75;

    //FAR VALUES AREN'T TESTED YET
    private double minFarFlywheelRPM = 1550;

    private double targetFarFlywheelRPM = 1580;

    //-----------------------INTAKE CONSTANTS------------------------------
    private double intakePower = 0.6;

    //---------------------------PUSHERUPPER CONSTANTS-------------------------
    private double pusherPower = 0.8;

    private double pusherTimer = 0.55;


    public void init(HardwareMap hwMap){
         intake = hwMap.get(DcMotorEx.class, "intake"); // control 3
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        shooter = hwMap.get(DcMotorEx.class, "launch"); // control 1
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(450, 0 ,0, 15);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        shooter2 = hwMap.get(DcMotorEx.class, "launch2"); // location unknown
        shooter2.setDirection(DcMotorEx.Direction.FORWARD);
        shooter2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients2 = new PIDFCoefficients(450, 0 ,0, 15);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients2);
        //TODO add PIDF tuning


         pusherupper = hwMap.get(DcMotorEx.class, "pusherupper"); // expand 2
        pusherupper.setDirection(DcMotorEx.Direction.FORWARD);
        pusherupper.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        pusherupper.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


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
                    shooter.setVelocity(targetFarFlywheelRPM);
                    shooter2.setVelocity(targetFarFlywheelRPM);
                    intake.setPower(intakePower);

                    stateTimer.reset();
                    flywheelState = FlywheelState.SPIN;
                }
                break;
            case SPIN:
                if (flywheelVelocity > minFarFlywheelRPM || stateTimer.seconds() > flywheelMaxRevTime){
                    ramp.setPosition(rampUpPosition);
                    pusherupper.setPower(pusherPower);

                    stateTimer.reset();
                    flywheelState = FlywheelState.LAUNCH;
                }
                break;
            case LAUNCH:
                if (stateTimer.seconds() > rampUpTime){
                    ramp.setPosition(rampDownPosition);
                    shotsRemain -= 1;

                    stateTimer.reset();
                    flywheelState = FlywheelState.RESET;
                }
                break;
            case RESET:
                shooter.setVelocity(0);
                shooter2.setVelocity(0);
                ramp.setPosition(rampDownPosition);
                pusherupper.setPower(0);

                stateTimer.reset();
                flywheelState = FlywheelState.IDLE;
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
