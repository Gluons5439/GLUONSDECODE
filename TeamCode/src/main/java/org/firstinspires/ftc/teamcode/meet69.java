
/*
package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "69PLZ?")
public class meet69 extends OpMode {

    // --- Pedro Pathing V2 Variables ---
    private Follower follower;
    private final double x = 8;
    private final double y = 66;
    private final Pose startPose = new Pose(x,y,0);

    private final double xf = 120;
    private final double yf = 0;
    double dx;
    double dy;

    // --- Limelight ---
    private Limelight3A limelight;
    private LLResult limelightResult;
    private static double LIMELIGHT_K = 0.8; // tuning for Limelight influence

    // --- Turret System Variables ---
    private DcMotorEx turretMoveMotor;
    private double turretMovePower = 0;
    private double kP = 28, kI = 0.05, kD = 2, kF = 7;
    private double error = 0, integralSum = 0, derivative = 0, lastError = 0;
    private double headingNeed = 0;
    private double degrees = 0;
    private double[] stepSizes = {10.0 , 1.0 , 0.1 , 0.01, 0.001};
    int stepIndex = 2;
    private boolean autoTurretEnabled = true;   // start ON
    private boolean x2_was_pressed = false;

    ElapsedTime timer = new ElapsedTime();

    // --- Shooter Variables ---
    private DcMotorEx shooterMotor;
    private DcMotorEx shooterMotor2;
    private boolean a_was_pressed = false;
    private boolean b_was_pressed = false;
    private enum ShooterMode {CLOSE, FAR}
    private ShooterMode shooterMode = ShooterMode.CLOSE;
    private boolean shooterActive = false;
    public double closeZoneTargetVelocity = 1800;
    public double farZoneTargetVelocity = 2100;

    // --- Intake Variables ---
    private DcMotor intakeMotor;
    private static final double INTAKE_POWER = -1.0;

    // --- Transfer Servo 1 (Cycling Mechanism) ---
    private Servo transferServo;
    private static final double TRANSFER_DOWN_POS = 0.5;
    private static final double TRANSFER_UP_POS = 0.85;

    private enum TransferState {REST, PUSHING, RETURNING, INTAKE_BURST}
    private TransferState currentTransferState = TransferState.REST;
    private double pushStartTime = 0.0;
    private static final double PUSH_DURATION_SECONDS = 0.300;
    private double burstStartTime = 0.0;
    private static final double INTAKE_BURST_DURATION = 0.250;
    private boolean dpadUp2_was_pressed = false;

    // --- Transfer Servo 2 (Stopper/Gate) ---
    private Servo StopperServo;
    private static final double STOPPER_DOWN_POS = 0.267;
    private static final double STOPPER_UP_POS = 0.05;

    @Override
    public void init() {

        // --- Follower init ---
        follower = Constants.createFollower(hardwareMap);
        if (follower != null) {
            follower.setStartingPose(startPose);
            follower.update();
        }

        // --- Limelight ---
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        // --- Turret motor ---
        turretMoveMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMoveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        PIDFCoefficients pidfCoefficientsTurret = new PIDFCoefficients(kP,kI,kD,kF);
        turretMoveMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficientsTurret);
        turretMoveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMoveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Shooter & Intake ---
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");

        PIDFCoefficients pidfShooter = new PIDFCoefficients(0.4,0,0,13.2);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfShooter);
        shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfShooter);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // --- Transfer servos ---
        transferServo = hardwareMap.get(Servo.class, "TransferServo");
        transferServo.setPosition(TRANSFER_DOWN_POS);
        StopperServo = hardwareMap.get(Servo.class, "StopperServo");
        StopperServo.setPosition(STOPPER_DOWN_POS);

        telemetry.update();
    }

    @Override
    public void start() {
        if (follower != null) follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        // --- Drive ---
        if (follower != null) {
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
        if (x2Pressed && !x2_was_pressed) autoTurretEnabled = !autoTurretEnabled;
        x2_was_pressed = x2Pressed;

        // --- Turret control ---
        if(autoTurretEnabled) handleAutoTurretControl();
        else handleTurretControl();

        turretMoveMotor.setVelocity(turretMovePower);

        // --- Mechanisms ---
        handleShooterControl();
        handleIntakeControl();
        handleTransferControl();

        // --- Telemetry ---
        telemetry.addData("Shooter Active", shooterActive);
        telemetry.addData("Turret Power", turretMovePower);
        telemetry.addData("PID Error", error);
        telemetry.addData("Heading Target", headingNeed);
        telemetry.addData("Turret Angle", degrees);
        telemetry.update();
    }

    // ------------------ AUTO TURRET w/ PID + Limelight ------------------
    private void handleAutoTurretControl(){

        // PID target based on field coordinates
        dx = xf - follower.getPose().getX();
        dy = (144 - yf) - follower.getPose().getY();
        headingNeed = (Math.atan2(dy, dx) - follower.getHeading()) * (180/Math.PI);
        degrees = getAngle(turretMoveMotor.getCurrentPosition());
        error = headingNeed - degrees;

        // Add Limelight correction if valid
        limelightResult = limelight.getLatestResult();
        if(limelightResult != null && limelightResult.isValid()){
            double tx = limelightResult.getTx();
            error += LIMELIGHT_K * tx; // adjust PID error
        }

        integralSum += error * timer.seconds();
        derivative = error - lastError;
        turretMovePower = (kP * error) + (kI * integralSum) + (kD * derivative);
        lastError = error;
        timer.reset();
    }

    // ------------------ MANUAL TURRET ------------------
    private void handleTurretControl(){
        double joystickInput = gamepad2.left_stick_x;
        turretMovePower = joystickInput * 500;
    }

    // ------------------ SHOOTER CONTROL ------------------
    private void handleShooterControl() {
        if (shooterMotor == null || shooterMotor2 == null || StopperServo == null) return;

        boolean aPressed = gamepad2.a;
        boolean bPressed = gamepad2.b;

        if(bPressed && !b_was_pressed){
            if(shooterActive && shooterMode == ShooterMode.CLOSE) shooterActive = false;
            else { shooterActive = true; shooterMode = ShooterMode.CLOSE; }
        }

        if(aPressed && !a_was_pressed){
            if(shooterActive && shooterMode == ShooterMode.FAR) shooterActive = false;
            else { shooterActive = true; shooterMode = ShooterMode.FAR; }
        }

        a_was_pressed = aPressed;
        b_was_pressed = bPressed;

        double targetVelocity = shooterActive ? (shooterMode == ShooterMode.CLOSE ? closeZoneTargetVelocity : farZoneTargetVelocity) : 0;
        shooterMotor.setVelocity(targetVelocity);
        shooterMotor2.setVelocity(targetVelocity);

        StopperServo.setPosition(shooterActive ? STOPPER_UP_POS : STOPPER_DOWN_POS);
    }

    // ------------------ INTAKE CONTROL ------------------
    private void handleIntakeControl(){
        if(intakeMotor == null) return;

        boolean intakeForward = gamepad1.right_bumper || gamepad2.right_bumper;
        boolean intakeReverse = gamepad1.left_bumper || gamepad2.left_bumper;

        if(intakeForward) intakeMotor.setPower(INTAKE_POWER);
        else if(intakeReverse) intakeMotor.setPower(0.25);
        else if(currentTransferState != TransferState.INTAKE_BURST) intakeMotor.setPower(0); // allow transfer burst to override
    }

    // ------------------ TRANSFER CONTROL ------------------
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
                intakeMotor.setPower(INTAKE_POWER);
                if(getRuntime() >= burstStartTime + INTAKE_BURST_DURATION){
                    intakeMotor.setPower(0);
                    currentTransferState = TransferState.REST;
                }
                break;
        }
        dpadUp2_was_pressed = dpad_up_pressed;
    }

    public double getAngle(double encoderPosition){
        double kV = 963/180.0;
        return encoderPosition/kV;
    }

    @Override
    public void stop() {
        if (shooterMotor != null) shooterMotor.setPower(0);
        if (shooterMotor2 != null) shooterMotor2.setPower(0);
        if (intakeMotor != null) intakeMotor.setPower(0);
        if (turretMoveMotor != null) turretMoveMotor.setPower(0);
        if (transferServo != null) transferServo.setPosition(TRANSFER_DOWN_POS);
        if (StopperServo != null) StopperServo.setPosition(STOPPER_DOWN_POS);
    }
}
*/
package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "69PLZ")
public class meet69 extends OpMode {

    // --- Pedro Pathing V2 Variables ---
    private Follower follower;
    private final double x = 8;
    private final double y = 66;
    private final Pose startPose = new Pose(x, y, 0);
    private final double xf = 120;
    private final double yf = 0;
    double dx;
    double dy;

    // --- Limelight ---
    private Limelight3A limelight;
    private LLResult limelightResult;
    private static double LIMELIGHT_K = 0.8; // Limelight influence
    private boolean limelightValid = false;
    private int limelightValidFrames = 0;
    private static final int LIMELIGHT_FRAME_THRESHOLD = 3;
    private double filteredTx = 0; // Low-pass filter for Limelight

    // --- Turret Variables ---
    private DcMotorEx turretMoveMotor;
    private double turretMovePower = 0;
    private double kP = 28, kI = 0.05, kD = 2, kF = 7;
    private double error = 0, integralSum = 0, derivative = 0, lastError = 0;
    private double headingNeed = 0;
    private double degrees = 0;
    private double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};
    private int stepIndex = 2;
    private boolean autoTurretEnabled = true;
    private boolean x2_was_pressed = false;
    private ElapsedTime timer = new ElapsedTime();

    // --- Shooter Variables ---
    private DcMotorEx shooterMotor;
    private DcMotorEx shooterMotor2;
    private boolean a_was_pressed = false;
    private boolean b_was_pressed = false;
    private enum ShooterMode {CLOSE, FAR}
    private ShooterMode shooterMode = ShooterMode.CLOSE;
    private boolean shooterActive = false;
    public double closeZoneTargetVelocity = 1800;
    public double farZoneTargetVelocity = 2100;

    // --- Intake Variables ---
    private DcMotor intakeMotor;
    private static final double INTAKE_POWER = -1.0;

    // --- Transfer Servo 1 (Cycling) ---
    private Servo transferServo;
    private static final double TRANSFER_DOWN_POS = 0.5;
    private static final double TRANSFER_UP_POS = 0.85;
    private enum TransferState {REST, PUSHING, RETURNING, INTAKE_BURST}
    private TransferState currentTransferState = TransferState.REST;
    private double pushStartTime = 0.0;
    private static final double PUSH_DURATION_SECONDS = 0.300;
    private double burstStartTime = 0.0;
    private static final double INTAKE_BURST_DURATION = 0.250;
    private boolean dpadUp2_was_pressed = false;

    // --- Transfer Servo 2 (Stopper/Gate) ---
    private Servo StopperServo;
    private static final double STOPPER_DOWN_POS = 0.267;
    private static final double STOPPER_UP_POS = 0.05;

    @Override
    public void init() {

        // --- Follower init ---
        follower = Constants.createFollower(hardwareMap);
        if (follower != null) {
            follower.setStartingPose(startPose);
            follower.update();
        }

        // --- Limelight ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        // --- Turret motor ---
        turretMoveMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMoveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        PIDFCoefficients pidfCoefficientsTurret = new PIDFCoefficients(kP, kI, kD, kF);
        turretMoveMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficientsTurret);
        turretMoveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMoveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Shooter & Intake ---
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");

        PIDFCoefficients pidfShooter = new PIDFCoefficients(0.4, 0, 0, 13.2);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfShooter);
        shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfShooter);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // --- Transfer servos ---
        transferServo = hardwareMap.get(Servo.class, "TransferServo");
        transferServo.setPosition(TRANSFER_DOWN_POS);
        StopperServo = hardwareMap.get(Servo.class, "StopperServo");
        StopperServo.setPosition(STOPPER_DOWN_POS);

        telemetry.update();
    }

    @Override
    public void start() {
        if (follower != null) follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        // --- Drive ---
        if (follower != null) {
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
        if (x2Pressed && !x2_was_pressed) autoTurretEnabled = !autoTurretEnabled;
        x2_was_pressed = x2Pressed;

        // --- Turret control ---
        if (autoTurretEnabled) handleAutoTurretControl();
        else handleTurretControl();

        turretMoveMotor.setVelocity(turretMovePower);

        // --- Mechanisms ---
        handleShooterControl();
        handleIntakeControl();
        handleTransferControl();

        // --- Telemetry ---
        telemetry.addData("Shooter Active", shooterActive);
        telemetry.addData("Turret Power", turretMovePower);
        telemetry.addData("PID Error", error);
        telemetry.addData("Heading Target", headingNeed);
        telemetry.addData("Turret Angle", degrees);
        telemetry.addData("Limelight Valid", limelightValid);
        telemetry.update();
    }

    // ------------------ AUTO TURRET w/ PID + Limelight ------------------
    private void handleAutoTurretControl() {

        // Field-centric target
        dx = xf - follower.getPose().getX();
        dy = (144 - yf) - follower.getPose().getY();
        headingNeed = (Math.atan2(dy, dx) - follower.getHeading()) * (180 / Math.PI);
        degrees = getAngle(turretMoveMotor.getCurrentPosition());

        // Limelight
        limelightResult = limelight.getLatestResult();
        if (limelightResult != null && limelightResult.isValid()) {
            limelightValidFrames++;
            if (limelightValidFrames >= LIMELIGHT_FRAME_THRESHOLD) limelightValid = true;
            filteredTx = 0.7 * filteredTx + 0.3 * limelightResult.getTx(); // smooth noise
        } else {
            limelightValidFrames = 0;
            limelightValid = false;
        }

        if (limelightValid) {
            // Use Limelight as primary aiming
            error = filteredTx;
            integralSum = 0; // reset PID integral to avoid wind-up
        } else {
            // Fallback PID aiming to field target
            error = headingNeed - degrees;
            integralSum += error * timer.seconds();

            // Optional: rotate robot toward target to reacquire Limelight
            double robotTurnPower = Range.clip(error * 0.01, -0.3, 0.3);
            follower.setTeleOpDrive(0, 0, robotTurnPower, false);
        }

        derivative = error - lastError;
        turretMovePower = Range.clip(kP * error + kI * integralSum + kD * derivative, -600, 600);
        lastError = error;
        timer.reset();
    }

    // ------------------ MANUAL TURRET ------------------
    private void handleTurretControl() {
        double joystickInput = gamepad2.left_stick_x;
        turretMovePower = joystickInput * 500;
    }

    // ------------------ SHOOTER CONTROL ------------------
    private void handleShooterControl() {
        if (shooterMotor == null || shooterMotor2 == null || StopperServo == null) return;

        boolean aPressed = gamepad2.a;
        boolean bPressed = gamepad2.b;

        if (bPressed && !b_was_pressed) {
            if (shooterActive && shooterMode == ShooterMode.CLOSE) shooterActive = false;
            else { shooterActive = true; shooterMode = ShooterMode.CLOSE; }
        }

        if (aPressed && !a_was_pressed) {
            if (shooterActive && shooterMode == ShooterMode.FAR) shooterActive = false;
            else { shooterActive = true; shooterMode = ShooterMode.FAR; }
        }

        a_was_pressed = aPressed;
        b_was_pressed = bPressed;

        double targetVelocity = shooterActive ? (shooterMode == ShooterMode.CLOSE ? closeZoneTargetVelocity : farZoneTargetVelocity) : 0;
        shooterMotor.setVelocity(targetVelocity);
        shooterMotor2.setVelocity(targetVelocity);

        StopperServo.setPosition(shooterActive ? STOPPER_UP_POS : STOPPER_DOWN_POS);
    }

    // ------------------ INTAKE CONTROL ------------------
    private void handleIntakeControl() {
        if (intakeMotor == null) return;

        boolean intakeForward = gamepad1.right_bumper || gamepad2.right_bumper;
        boolean intakeReverse = gamepad1.left_bumper || gamepad2.left_bumper;

        if (intakeForward) intakeMotor.setPower(INTAKE_POWER);
        else if (intakeReverse) intakeMotor.setPower(0.25);
        else if (currentTransferState != TransferState.INTAKE_BURST) intakeMotor.setPower(0); // allow transfer burst
    }

    // ------------------ TRANSFER CONTROL ------------------
    private void handleTransferControl() {
        boolean dpad_up_pressed = gamepad2.y;

        switch (currentTransferState) {
            case REST:
                if (dpad_up_pressed && !dpadUp2_was_pressed) {
                    transferServo.setPosition(TRANSFER_UP_POS);
                    pushStartTime = getRuntime();
                    currentTransferState = TransferState.PUSHING;
                }
                break;
            case PUSHING:
                if (getRuntime() >= pushStartTime + PUSH_DURATION_SECONDS) {
                    transferServo.setPosition(TRANSFER_DOWN_POS);
                    currentTransferState = TransferState.RETURNING;
                }
                break;
            case RETURNING:
                burstStartTime = getRuntime();
                currentTransferState = TransferState.INTAKE_BURST;
                break;
            case INTAKE_BURST:
                intakeMotor.setPower(INTAKE_POWER);
                if (getRuntime() >= burstStartTime + INTAKE_BURST_DURATION) {
                    intakeMotor.setPower(0);
                    currentTransferState = TransferState.REST;
                }
                break;
        }
        dpadUp2_was_pressed = dpad_up_pressed;
    }

    public double getAngle(double encoderPosition) {
        double kV = 963 / 180.0;
        return encoderPosition / kV;
    }

    @Override
    public void stop() {
        if (shooterMotor != null) shooterMotor.setPower(0);
        if (shooterMotor2 != null) shooterMotor2.setPower(0);
        if (intakeMotor != null) intakeMotor.setPower(0);
        if (turretMoveMotor != null) turretMoveMotor.setPower(0);
        if (transferServo != null) transferServo.setPosition(TRANSFER_DOWN_POS);
        if (StopperServo != null) StopperServo.setPosition(STOPPER_DOWN_POS);
    }
}
