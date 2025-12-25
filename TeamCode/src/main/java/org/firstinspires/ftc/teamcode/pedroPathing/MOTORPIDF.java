package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
@TeleOp(name = "MOTOR PIDF TUNING")
public class MOTORPIDF extends OpMode{

    public DcMotorEx flywheelMotor1;
    public DcMotorEx flywheelMotor2;

    public double highVelocity = 1500;

    public double lowVelocity = 900;

    double curTargetVelocity = highVelocity;

    double F= 0;
    double P = 0;

    double[] stepSizes = {10.0 , 1.0 , 0.1 , 0.01, 0.001};
    int stepIndex = 1;

    @Override
    public void init()
    {
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        flywheelMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        PIDFCoefficients pidfCoefficents = new PIDFCoefficients(P,0,0,F);
        flywheelMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficents);
        flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficents);
        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop(){
        if(gamepad1.yWasPressed())
        {
            if(curTargetVelocity==highVelocity)
            {
                curTargetVelocity=lowVelocity;
            } else {
                curTargetVelocity=highVelocity;
            }
        }
        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex +1)% stepSizes.length;
        }

        if(gamepad1.dpadLeftWasPressed())
        {
            F -= stepSizes[stepIndex];
        }
        if(gamepad1.dpadRightWasPressed())
        {
            F+= stepSizes[stepIndex];
        }
        if(gamepad1.dpadUpWasPressed())
        {
            P-= stepSizes[stepIndex];
        }
        if(gamepad1.dpadDownWasPressed())
        {
            P+= stepSizes[stepIndex];
        }


        PIDFCoefficients pidfCoefficents = new PIDFCoefficients(P,0,0,F);
        flywheelMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficents);
        flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficents);

        flywheelMotor1.setVelocity(curTargetVelocity);
        flywheelMotor2.setVelocity(curTargetVelocity);

        double curVelocity1 = flywheelMotor1.getVelocity();
        double curVelocity2 = flywheelMotor2.getVelocity();
        double error1 = curTargetVelocity-curVelocity1;
        double error2 = curTargetVelocity-curVelocity2;

        telemetry.addData("Target Velocity: ", curTargetVelocity);
        telemetry.addData("Motor1 Current Velocity: ","%.2f", curVelocity1);
        telemetry.addData("Motor2 Current Velocity: ","%.2f", curVelocity2);
        telemetry.addData("Error Motor 1: ", error1);
        telemetry.addData("Error Motor 2: ", error2);
        telemetry.addLine("--------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P );
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F );
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);



    }
}
