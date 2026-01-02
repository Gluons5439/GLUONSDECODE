package org.firstinspires.ftc.teamcode.Auto.Mechanisms;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeLogic {
    DcMotor intakeMotor;
    private ElapsedTime timer = new ElapsedTime();
    public void init(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0);
    }
    public void setZaPower(double power)
    {
        intakeMotor.setPower(power);
    }
    public void set2sec(double power)
    {
        timer.reset();
        while(2>timer.seconds())
        {
            intakeMotor.setPower(power);
        }
        intakeMotor.setPower(0);
    }
}
