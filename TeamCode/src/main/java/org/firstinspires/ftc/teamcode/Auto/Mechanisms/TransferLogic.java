package org.firstinspires.ftc.teamcode.Auto.Mechanisms;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BlueTeleop;

public class TransferLogic {
    private enum TransferState {
        REST,
        PUSHING,
        RETURNING,
        INTAKE_BURST
    }
    public TransferState currentTransferState;
    public boolean transferNow = false;
    private Servo transferServo;
    private final double TRANSFER_DOWN_POS = 0.5;
    private final double TRANSFER_UP_POS = 0.85;
    private static final double PUSH_DURATION_SECONDS = 0.300;
    private static final double INTAKE_BURST_DURATION = 0.250;
    private static final double INTAKE_POWER = -1.0;
    private IntakeLogic intake;

    private ElapsedTime stateTimer = new ElapsedTime();
    public void init(HardwareMap hardwareMap,IntakeLogic intakeLogic){
        transferServo = hardwareMap.get(Servo.class, "TransferServo");
        transferServo.setPosition(TRANSFER_DOWN_POS);
        currentTransferState = TransferState.REST;
        intake = intakeLogic;
//        intake.init(hardwareMap);

    }
    public void fah(double time) {
        stateTimer.reset();
        while(stateTimer.seconds()<=time) {
            intake.setZaPower(0.9);
        }
        intake.setZaPower(0);
    }
    public void update() {

        switch (currentTransferState) {
            case REST:

                if (!transferNow) {
                    intake.setZaPower(0);
                }else {
                    transferServo.setPosition(TRANSFER_UP_POS);
                    stateTimer.reset();
                    currentTransferState = TransferState.PUSHING;
                }
                break;

            case PUSHING:
                if (stateTimer.seconds() >= PUSH_DURATION_SECONDS) {
                    transferServo.setPosition(TRANSFER_DOWN_POS);
                    currentTransferState = TransferState.RETURNING;
                }
                break;

            case RETURNING:
                stateTimer.reset();
                currentTransferState = TransferState.INTAKE_BURST;
                break;

            case INTAKE_BURST:
                intake.setZaPower(INTAKE_POWER);

                if (stateTimer.seconds() >= INTAKE_BURST_DURATION) {
                    intake.setZaPower(0);
                    currentTransferState = TransferState.REST;
                }
                break;
        }
       transferNow = false;


    }
    public void transfer()
    {
        if(currentTransferState == currentTransferState.REST)
        {
            transferNow = true;
        }

    }
    public boolean isBusy(){
        return currentTransferState != currentTransferState.REST;
    }
}
