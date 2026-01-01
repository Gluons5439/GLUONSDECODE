


package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "Limelight + Transfer")
public class FixedLimelightTracking extends OpMode {

    // --- Pedro Pathing ---
    private Follower follower;
    private final Pose startPose = new Pose(8, 66, 0);

    // --- Limelight ---
    private Limelight3A limelight;
    public static double AUTO_AIM_K = 25.0;        // ticks/sec per deg
    public static double MAX_TURRET_VEL = 800.0;   // ticks/sec
    public static double LIMELIGHT_DEADBAND = 0.8; // deg

    // --- Turret ---
    private DcMotorEx turretMoveMotor;
    private double turretMoveVelocity = 0;
    private boolean autoTurretEnabled = true;
    private boolean x2_was_pressed = false;

    // --- Shooter ---
    private DcMotorEx shooterMotor;
    private DcMotorEx shooterMotor2;
    private enum ShooterMode {CLOSE, FAR}
    private ShooterMode shooterMode = ShooterMode.CLOSE;
    private boolean shooterActive = false;
    public static double closeZoneTargetVelocity = 1800;
    public static double farZoneTargetVelocity = 2100;
    private boolean a_was_pressed = false;
    private boolean b_was_pressed = false;

    // --- Intake ---
    private DcMotor intakeMotor;
    private static final double INTAKE_POWER = -1.0;

    // --- Transfer ---
    private Servo transferServo;
    private Servo StopperServo;
    private static final double TRANSFER_DOWN_POS = 0.5;
    private static final double TRANSFER_UP_POS = 0.85;
    private static final double STOPPER_DOWN_POS = 0.267;
    private static final double STOPPER_UP_POS = 0.05;

    private enum TransferState {REST, PUSHING, RETURNING, INTAKE_BURST}
    private TransferState currentTransferState = TransferState.REST;
    private double pushStartTime = 0.0;
    private static final double PUSH_DURATION_SECONDS = 0.300;
    private double burstStartTime = 0.0;
    private static final double INTAKE_BURST_DURATION = 0.250;
    private boolean dpadUp2_was_pressed = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        if(follower != null){
            follower.setStartingPose(startPose);
            follower.update();
        }

        turretMoveMotor = hardwareMap.get(DcMotorEx.class,"turretMotor");
        turretMoveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMoveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMoveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMoveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        shooterMotor = hardwareMap.get(DcMotorEx.class,"shooterMotor");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class,"shooterMotor2");
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor = hardwareMap.get(DcMotor.class,"Intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        transferServo = hardwareMap.get(Servo.class,"TransferServo");
        StopperServo = hardwareMap.get(Servo.class,"StopperServo");

        transferServo.setPosition(TRANSFER_DOWN_POS);
        StopperServo.setPosition(STOPPER_DOWN_POS);
    }

    @Override
    public void start(){
        if(follower != null) follower.startTeleopDrive();
    }

    @Override
    public void loop(){
        // --- Drive ---
        if(follower != null){
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false
            );
            follower.update();
        }

        // --- Toggle auto turret ---
        boolean x2Pressed = gamepad2.x;
        if(x2Pressed && !x2_was_pressed) autoTurretEnabled = !autoTurretEnabled;
        x2_was_pressed = x2Pressed;

        // --- Turret ---
        if(autoTurretEnabled) handleAutoTurretControl();
        else handleManualTurretControl();
        turretMoveMotor.setVelocity(turretMoveVelocity);

        // --- Mechanisms ---
        handleShooterControl();
        handleTransferControl();
        handleIntakeControl();

        // --- Telemetry ---
        telemetry.addData("Auto Turret", autoTurretEnabled);
        telemetry.addData("Turret Vel", turretMoveVelocity);
        telemetry.addData("Transfer State", currentTransferState);
        telemetry.addData("Shooter Active", shooterActive);
        telemetry.update();
    }

    // --- Auto Turret ---
    private void handleAutoTurretControl(){
        LLResult limelightResult = limelight.getLatestResult();
        if(limelightResult == null || !limelightResult.isValid()){ turretMoveVelocity = 0; return; }

        double tx = limelightResult.getTx();
        if(Math.abs(tx) < LIMELIGHT_DEADBAND){ turretMoveVelocity = 0; return; }

        turretMoveVelocity = Range.clip(AUTO_AIM_K * tx, -MAX_TURRET_VEL, MAX_TURRET_VEL);
    }

    private void handleManualTurretControl(){
        turretMoveVelocity = gamepad2.left_stick_x * MAX_TURRET_VEL;
    }

    // --- Shooter ---
    private void handleShooterControl(){
        boolean aPressed = gamepad2.a;
        boolean bPressed = gamepad2.b;

        if(bPressed && !b_was_pressed){
            shooterActive = !(shooterActive && shooterMode == ShooterMode.CLOSE);
            shooterMode = ShooterMode.CLOSE;
        }
        if(aPressed && !a_was_pressed){
            shooterActive = !(shooterActive && shooterMode == ShooterMode.FAR);
            shooterMode = ShooterMode.FAR;
        }
        a_was_pressed = aPressed;
        b_was_pressed = bPressed;

        double targetVelocity = shooterActive ? (shooterMode == ShooterMode.CLOSE ? closeZoneTargetVelocity : farZoneTargetVelocity) : 0;
        shooterMotor.setVelocity(targetVelocity);
        shooterMotor2.setVelocity(targetVelocity);

        StopperServo.setPosition(shooterActive ? STOPPER_UP_POS : STOPPER_DOWN_POS);
    }

    // --- Intake ---
    private void handleIntakeControl(){
        double intakePower = 0;
        if(gamepad1.right_bumper || gamepad2.right_bumper) intakePower = INTAKE_POWER;
        else if(gamepad1.left_bumper || gamepad2.left_bumper) intakePower = 0.25;

        // Override during transfer burst
        if(currentTransferState == TransferState.INTAKE_BURST) intakePower = INTAKE_POWER;

        intakeMotor.setPower(intakePower);
    }

    // --- Transfer ---
    private void handleTransferControl(){
        boolean dpad_up_pressed = gamepad2.y;

        switch(currentTransferState){
            case REST:
                if(dpad_up_pressed && !dpadUp2_was_pressed){
                    transferServo.setPosition(TRANSFER_UP_POS);
                    pushStartTime = getRuntime();
                    currentTransferState = TransferState.PUSHING;
                }
                break;

            case PUSHING:
                if(getRuntime() >= pushStartTime + PUSH_DURATION_SECONDS){
                    transferServo.setPosition(TRANSFER_DOWN_POS);
                    currentTransferState = TransferState.RETURNING;
                }
                break;

            case RETURNING:
                burstStartTime = getRuntime();
                currentTransferState = TransferState.INTAKE_BURST;
                break;

            case INTAKE_BURST:
                if(getRuntime() >= burstStartTime + INTAKE_BURST_DURATION){
                    currentTransferState = TransferState.REST;
                }
                break;
        }

        dpadUp2_was_pressed = dpad_up_pressed;
    }
}
