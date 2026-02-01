package org.firstinspires.ftc.teamcode.teleop.launchtesting;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@TeleOp
public class PIDFFlywheelTuning extends OpMode{

    public DcMotorEx flywheelMotor;

    public double highVelocity = 1500;
    public double lowVelocity = 1250;

    double curTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};

    int stepIndex = 1;

    @Override
    public void init(){
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "launch");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.07,0,0.1, 0.04);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addLine("init complete");


    }

    @Override
    public void loop(){
        //Get gamepad commands and set target velocity, and update telemetry

        if(gamepad2.yWasPressed()){
            if (curTargetVelocity == highVelocity){
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }

        }
        if (gamepad2.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad2.dpadLeftWasPressed()){
            F -= stepSizes[stepIndex];
        }
        if (gamepad2.dpadRightWasPressed()){
            F += stepSizes[stepIndex];
        }


        if (gamepad2.dpadDownWasPressed()){
            P -= stepSizes[stepIndex];
        }
        if (gamepad2.dpadUpWasPressed()){
            P += stepSizes[stepIndex];
        }


        //Set new PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0 ,0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        //SET VELOCITY
        flywheelMotor.setVelocity(curTargetVelocity);

        double curVelocity = flywheelMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("------------------------------------");
        telemetry.addData("Tuning P", "%.4f (DPad Up/Down", P);
        telemetry.addData("Tuning F", "%.4f (DPad Left/Right", F);
        telemetry.addData("Step Size", "%.4f (B Button", stepSizes[stepIndex]);


    }


}
