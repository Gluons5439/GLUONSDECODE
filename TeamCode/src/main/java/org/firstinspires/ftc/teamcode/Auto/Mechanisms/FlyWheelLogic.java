package org.firstinspires.ftc.teamcode.Auto.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FlyWheelLogic {
    private DcMotorEx shooterMotor;
    private DcMotorEx shooterMotor2;
    private Servo transferServo;
    private ElapsedTime stateTimer = new ElapsedTime();

    private enum FlywheelState {
        IDLE,
        LAUNCH,
        RESET_GATE
    }

    private FlywheelState flywheelState;

    private final double TRANSFER_DOWN_POS = 0.5;
    private final double TRANSFER_UP_POS = 0.85;
    private double TRANSFER_TIME = 0.3;


    private int shotsRemaining = 0;
    private double flywheelVelocity = 0;
    private double closeZoneTargetVelocity = 1800;
    private double farZoneTargetVelocity = 2100;
    private double maxSpinupTime = 2;

    private TransferLogic transferLogic;

    public void init(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        transferServo = hardwareMap.get(Servo.class, "TransferServo");
        transferLogic = new TransferLogic();
        transferLogic.init(hardwareMap);
        PIDFCoefficients pidfCoefficentsShooter = new PIDFCoefficients(0.4,0,0,13.2);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficentsShooter);
        shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficentsShooter);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        flywheelState = FlywheelState.IDLE;
        shooterMotor.setVelocity(0);
        shooterMotor2.setVelocity(0);
        transferServo.setPosition(TRANSFER_DOWN_POS);
    }

    public void update() {
        switch(flywheelState) {
            case IDLE:
                if(shotsRemaining>0) {
                    shooterMotor.setVelocity(farZoneTargetVelocity);
                    shooterMotor2.setVelocity(farZoneTargetVelocity);
                    stateTimer.reset();
                    flywheelState = FlywheelState.LAUNCH;
                }
                break;
            case LAUNCH:
                if((flywheelVelocity > closeZoneTargetVelocity || stateTimer.seconds() > maxSpinupTime)&&!transferLogic.isBusy()) {
                    transferLogic.transfer();
                    stateTimer.reset();
                    flywheelState = FlywheelState.RESET_GATE;
                }
                break;
            case RESET_GATE:
                if(!transferLogic.isBusy()) {
                    if (shotsRemaining > 0) {
                        stateTimer.reset();
                        flywheelState = flywheelState.LAUNCH;
                    } else {
                        shooterMotor.setVelocity(0);
                        shooterMotor2.setVelocity(0);
                        flywheelState = flywheelState.IDLE;
                    }
                }
                break;
        }
    }
    public void fireShots(int numberOfShots)
    {
        if(flywheelState == flywheelState.IDLE)
        {
            shotsRemaining = numberOfShots;
        }
    }
    public boolean isBusy(){
        return flywheelState != flywheelState.IDLE;
    }

}
