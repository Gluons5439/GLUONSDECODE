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
@TeleOp(name = "Working-LimelightTracking")
public class FixedLimelightTracking extends OpMode {

    // --- Pedro Pathing V2 Variables ---
    private Follower follower;
    private final double x = 8;
    private final double y = 66;
    private final Pose startPose = new Pose(x,y,0);

    // ==================== LIMELIGHT ====================
    private Limelight3A limelight;
    private LLResult limelightResult;

    public enum Alliance { RED, BLUE }
    public static Alliance ALLIANCE = Alliance.RED;

    private static final int RED_TAG_ID = 24;
    private static final int BLUE_TAG_ID = 20;

    // Vision PD gains - MATCHING WORKING CODE
    public static double visionKP = 0.05;
    public static double visionKD = 0.0002;

    public static double FAR_DISTANCE_THRESHOLD = 50.0;
    public static double FURTHER_CORRECTION = 3.0;

    // ==================== TURRET ====================
    private DcMotorEx turretMoveMotor;

    // Motor PID gains - MUCH LOWER LIKE WORKING CODE
    public static double kP = 0.015;  // Was 28 - way too high!
    public static double kD = 0.0008; // Was 2 - too high!
    public static double kF = 0.0;

    private static final double COUNTS_PER_REV = 537.7;
    private static final double COUNTS_PER_DEGREE = COUNTS_PER_REV / 360.0;

    public static double MAX_POWER = 0.8;  // Using power not velocity
    public static double MIN_ANGLE = -110;
    public static double MAX_ANGLE = 110;

    public static double TARGET_TOLERANCE = 2.0;

    private double targetAngle = 0;
    private double currentAngle = 0;
    private double turretPower = 0;

    // Timing variables
    private double lastTime = 0;
    private double period = 0;

    private double lastError = 0;

    private double visionLastTime = 0;
    private double visionPeriod = 0;
    private double visionLastError = 0;

    private boolean autoTurretEnabled = true;
    private boolean x2_was_pressed = false;

    // --- Shooter / Intake / Transfer ---
    private DcMotorEx shooterMotor;
    private DcMotorEx shooterMotor2;
    private DcMotor intakeMotor;

    private Servo transferServo;
    private Servo StopperServo;

    private static final double TRANSFER_DOWN_POS = 0.5;
    private static final double TRANSFER_UP_POS = 0.85;
    private static final double STOPPER_DOWN_POS = 0.267;
    private static final double STOPPER_UP_POS = 0.05;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        if (follower != null) {
            follower.setStartingPose(startPose);
            follower.update();
        }

        // CRITICAL: Use correct motor name and setup like working code
        turretMoveMotor = hardwareMap.get(DcMotorEx.class, "turretMotor"); // Changed from "turretMotor"
        turretMoveMotor.setDirection(DcMotorSimple.Direction.REVERSE); // Working code uses REVERSE
        turretMoveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMoveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Changed from RUN_USING_ENCODER
        turretMoveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");

        transferServo = hardwareMap.get(Servo.class, "TransferServo");
        StopperServo = hardwareMap.get(Servo.class, "StopperServo");

        transferServo.setPosition(TRANSFER_DOWN_POS);
        StopperServo.setPosition(STOPPER_DOWN_POS);

        // Initialize timing
        lastTime = System.nanoTime() / 1E9;
        visionLastTime = lastTime;
    }

    @Override
    public void start() {
        if (follower != null) follower.startTeleopDrive();
        lastTime = System.nanoTime() / 1E9;
        visionLastTime = lastTime;
    }

    @Override
    public void loop() {

        if (follower != null) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false
            );
            follower.update();
        }

        // Toggle auto turret with X button
        boolean x2Pressed = gamepad2.x;
        if (x2Pressed && !x2_was_pressed) {
            autoTurretEnabled = !autoTurretEnabled;
            lastError = 0;
            visionLastError = 0;
            lastTime = System.nanoTime() / 1E9;
            visionLastTime = lastTime;
        }
        x2_was_pressed = x2Pressed;

        if (autoTurretEnabled) {
            handleAutoTurretControl();
        } else {
            handleManualTurretControl();
        }

        // Use setPower() not setVelocity()
        turretMoveMotor.setPower(turretPower);

        // Telemetry
        telemetry.addData("=== TURRET STATUS ===", "");
        telemetry.addData("Auto Turret", autoTurretEnabled ? "ENABLED" : "MANUAL");
        telemetry.addData("Current Angle", "%.2f°", currentAngle);
        telemetry.addData("Target Angle", "%.2f°", targetAngle);
        telemetry.addData("Error", "%.2f°", targetAngle - currentAngle);
        telemetry.addData("Power", "%.2f", turretPower);
        telemetry.addData("At Target", Math.abs(targetAngle - currentAngle) < TARGET_TOLERANCE);

        telemetry.addData("=== LIMELIGHT ===", "");
        if (limelightResult != null && limelightResult.isValid()) {
            telemetry.addData("Target", "LOCKED");
            telemetry.addData("TX (offset)", "%.2f", limelightResult.getTx());
            telemetry.addData("TA (area)", "%.2f", limelightResult.getTa());
            if (!limelightResult.getFiducialResults().isEmpty()) {
                telemetry.addData("Tag ID", limelightResult.getFiducialResults().get(0).getFiducialId());
            }
        } else {
            telemetry.addData("Target", "SEARCHING");
        }

        telemetry.update();
    }

    // ==================== AUTO TURRET (LIMELIGHT) ====================

    private void handleAutoTurretControl() {

        // 1. UPDATE LIMELIGHT
        limelightResult = limelight.getLatestResult();

        // 2. READ CURRENT POSITION
        int encoderPosition = turretMoveMotor.getCurrentPosition();
        currentAngle = encoderPosition / COUNTS_PER_DEGREE;

        // 3. PROCESS VISION AND ADJUST TARGET
        if (hasValidTarget()) {
            double tx = limelightResult.getTx();
            double ta = limelightResult.getTa();

            double correctedTX = applyDistanceCorrection(tx, ta);

            // Vision PD with proper timing
            double visionError = correctedTX;

            double currentTime = System.nanoTime() / 1E9;
            visionPeriod = currentTime - visionLastTime;
            visionLastTime = currentTime;

            // Calculate derivative (delta / time)
            double visionDerivative = 0;
            if (visionPeriod > 1E-6) {
                visionDerivative = (visionError - visionLastError) / visionPeriod;
            }

            double adjustment = (visionKP * visionError) + (visionKD * visionDerivative);

            targetAngle -= adjustment;  // Subtract like working code
            visionLastError = visionError;

        } else {
            visionLastError = 0;
            visionLastTime = System.nanoTime() / 1E9;
        }

        // 4. LIMIT TARGET
        targetAngle = Range.clip(targetAngle, MIN_ANGLE, MAX_ANGLE);

        // 5. MAIN PID WITH PROPER TIMING
        double error = targetAngle - currentAngle;

        double currentTime = System.nanoTime() / 1E9;
        period = currentTime - lastTime;
        lastTime = currentTime;

        // Calculate derivative (delta / time)
        double derivative = 0;
        if (period > 1E-6) {
            derivative = (error - lastError) / period;
        }

        turretPower = (kP * error) + (kD * derivative) + kF;
        turretPower = Range.clip(turretPower, -MAX_POWER, MAX_POWER);

        lastError = error;
    }

    private void handleManualTurretControl() {
        int encoderPosition = turretMoveMotor.getCurrentPosition();
        currentAngle = encoderPosition / COUNTS_PER_DEGREE;

        // Manual control with joystick
        turretPower = -gamepad2.left_stick_x * MAX_POWER;

        // Update target to follow current position
        targetAngle = currentAngle;

        // Reset control states
        lastError = 0;
        visionLastError = 0;
        lastTime = System.nanoTime() / 1E9;
        visionLastTime = lastTime;
    }

    private boolean hasValidTarget() {
        if (limelightResult == null || !limelightResult.isValid()) return false;
        if (limelightResult.getFiducialResults().isEmpty()) return false;

        int id = limelightResult.getFiducialResults().get(0).getFiducialId();

        return (ALLIANCE == Alliance.RED && id == RED_TAG_ID) ||
                (ALLIANCE == Alliance.BLUE && id == BLUE_TAG_ID);
    }

    private double applyDistanceCorrection(double tx, double ta) {
        if (ta < FAR_DISTANCE_THRESHOLD) {
            return (ALLIANCE == Alliance.RED)
                    ? tx + FURTHER_CORRECTION
                    : tx - FURTHER_CORRECTION;
        }
        return tx;
    }
}