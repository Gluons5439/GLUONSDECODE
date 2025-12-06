



package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo; // Included
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "Final Field-Centric TeleOp")
public class TestFieldDrive3 extends OpMode {

    // --- Pedro Pathing V2 Variables ---
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);

    // --- Turret System Variables ---
    private Servo turretServo1;
    private Servo turretServo2;
    private double turretTargetPosition = 0.5;

    // --- Shooter Variables ---
    private DcMotor shooterMotor;
    private DcMotor shooterMotor2;
    private static final double SHOOTER_MAX_POWER = -2.0;
    private boolean shooterActive = false;

    // --- Intake Variables ---
    private DcMotor intakeMotor;
    private static final double INTAKE_POWER = -1.0;

    // --- Transfer Servo Variables (MODIFIED: Single Positional Servo) ---
    private Servo transferServo2;
    private int transferState = 0; // 0=Down, 1=Up/Delaying
    private long sequenceStartTime = 0;
    private static final long UP_DURATION_MS = 300; // Time (in milliseconds) the servo stays up
    private static final double TRANSFER_DOWN_POS = 0.0; // Resting/Down Position
    private static final double TRANSFER_UP_POS = 1.0;   // Flipped/Up Position

    // --- Vision/Limelight Variables ---
    private Limelight3A limelight;

    // --- Auto-Aim PID Constants ---
    private static final double AIM_KP = 0.0008;
    private static final double AIM_DEADZONE = 1.5;
    private static final double MAX_CORRECTION = 0.001;

    // --- Toggle variables ---
    private boolean isAutoAimActive = false;
    private boolean a_was_pressed = false;
    private boolean b_was_pressed = false; // For shooter toggle
    private boolean left_bumper2_was_pressed = false; // For transfer one-shot

    @Override
    public void init() {
        // --- PEDRO PATHING V2 INITIALIZATION ---
        follower = Constants.createFollower(hardwareMap);
        if (follower != null) {
            follower.setStartingPose(startPose);
            follower.update();
        }

        // --- HARDWARE INITIALIZATION ---

        // Initialize Turret Servos
        try {
            turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
            turretServo2 = hardwareMap.get(Servo.class, "turretServo2");
            turretServo1.setPosition(turretTargetPosition);
            turretServo2.setPosition(turretTargetPosition);
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find turret servos.");
        }

        // Initialize Shooter Motors
        try {
            shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            telemetry.addData("Error", "ShooterMotor 1 not found.");
        }

        try {
            shooterMotor2 = hardwareMap.get(DcMotor.class, "shooterMotor2");
            shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            telemetry.addData("Error", "ShooterMotor 2 not found.");
        }

        // Initialize Intake Motor
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find Intake motor.");
        }

        // Initialize Transfer Servo 2 (NEW)
        try {
            transferServo2 = hardwareMap.get(Servo.class, "Transfer2");
            transferServo2.setPosition(TRANSFER_DOWN_POS); // Set to resting position
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find Transfer Servo 2.");
        }

        // Initialize Limelight
        try {
            limelight = hardwareMap.get(Limelight3A.class, "Limelight");
            limelight.pipelineSwitch(0);
        } catch (Exception e) {
            telemetry.addData("Error", "Limelight not found.");
        }

        telemetry.update();
    }

    @Override
    public void start() {
        if (follower != null) follower.startTeleopDrive();
        if (limelight != null) limelight.start();
    }

    @Override
    public void loop() {
        // --- PEDRO PATHING V2 DRIVING (FIELD CENTRIC) ---
        if (follower != null) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false
            );
            follower.update();
        }

        // --- AUTO-AIM TOGGLE (GamePad2 A) ---
        boolean a_is_pressed = gamepad2.a;
        if (a_is_pressed && !a_was_pressed) {
            isAutoAimActive = !isAutoAimActive;
        }
        a_was_pressed = a_is_pressed;

        // --- TURRET ACTION LOGIC ---
        if (isAutoAimActive) {
            handleAutoAim();
        } else {
            handleTurretControl();
        }

        // Clip and apply the position to the servos
        turretTargetPosition = Range.clip(turretTargetPosition, 0.0, 1.0);
        if (turretServo1 != null && turretServo2 != null) {
            turretServo1.setPosition(turretTargetPosition);
            turretServo2.setPosition(turretTargetPosition);
        }

        // --- MECHANISM CONTROL ---
        handleShooterControl();   // Gamepad 2 B
        handleIntakeControl();    // Gamepad 1 Bumpers
        handleTransferControl();  // Gamepad 2 Left Bumper (RE-ADDED)

        // --- TELEMETRY ---
        telemetry.addData("Drive Mode", "FIELD CENTRIC");
        telemetry.addData("Turret Mode (A)", isAutoAimActive ? "AUTO-AIM" : "MANUAL (Left Stick X 2)");
        telemetry.addData("Shooter Active (B)", shooterActive);
        telemetry.addData("Intake Control", "R/L Bumpers 1");
        telemetry.addData("Transfer State", transferState == 0 ? "DOWN (Ready)" : "UP (Sequencing)");
        telemetry.addData("Robot Pose", follower != null ? follower.getPose() : "N/A");

        telemetry.update();
    }


    //
    // ---------- TRANSFER CONTROL (GamePad2 Left Bumper - ONE-SHOT) ----------
    //
    private void handleTransferControl() {
        if (transferServo2 == null) return;

        boolean left_bumper2_is_pressed = gamepad2.left_bumper;

        // --- State Machine Logic for Flip Up/Down ---

        // State 0: Down and Waiting for Press
        if (transferState == 0) {
            if (left_bumper2_is_pressed && !left_bumper2_was_pressed) {
                // 1. Flip Up
                transferServo2.setPosition(TRANSFER_UP_POS);
                sequenceStartTime = System.currentTimeMillis();
                transferState = 1; // Move to Up/Delay state
            }
        }

        // State 1: Up and Waiting for Timer
        else if (transferState == 1) {
            if (System.currentTimeMillis() >= sequenceStartTime + UP_DURATION_MS) {
                // 2. Timer elapsed, go back Down
                transferServo2.setPosition(TRANSFER_DOWN_POS);
                transferState = 0; // Return to Down/Ready state
            }
        }

        left_bumper2_was_pressed = left_bumper2_is_pressed; // Latch update
    }

    //
    // ---------- SHOOTER CONTROL (GamePad2 B - TOGGLE) ----------
    //
    private void handleShooterControl() {
        if (shooterMotor == null && shooterMotor2 == null) return;

        boolean b_is_pressed = gamepad2.b;

        if (b_is_pressed && !b_was_pressed) {
            shooterActive = !shooterActive;
        }
        b_was_pressed = b_is_pressed;

        double targetPower = shooterActive ? SHOOTER_MAX_POWER : 0.0;

        if (shooterMotor != null) shooterMotor.setPower(targetPower);
        if (shooterMotor2 != null) shooterMotor2.setPower(targetPower);
    }


    //
    // ---------- INTAKE CONTROL (GamePad1 Bumpers) ----------
    //
    private void handleIntakeControl() {
        if (intakeMotor == null) return;

        if (gamepad1.right_bumper) {
            intakeMotor.setPower(INTAKE_POWER);
        } else if (gamepad1.left_bumper) {
            intakeMotor.setPower(0.25);
        } else {
            intakeMotor.setPower(0);
        }
    }

    //
    // ---------- MANUAL TURRET CONTROL (GamePad2 Left Stick X) ----------
    //
    private void handleTurretControl() {
        double turretMoveInput = gamepad2.left_stick_x;
        turretTargetPosition = (turretMoveInput + 1.0) / 2.0;
    }

    //
    // ---------- AUTO-AIM USING LIMELIGHT (Proportional Control) ----------
    //
    private void handleAutoAim() {
        if (limelight == null) {
            telemetry.addLine("AUTO-AIM: Limelight not initialized!");
            return;
        }

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double tx = result.getTx();

            if (Math.abs(tx) > AIM_DEADZONE) {
                double correction = Range.clip(tx * AIM_KP, -MAX_CORRECTION, MAX_CORRECTION);
                turretTargetPosition -= correction;
                telemetry.addData("Target X (°)", "%.2f", tx);
                telemetry.addLine("Auto-Aim: Tracking...");
            } else {
                telemetry.addLine("Auto-Aim: Locked on!");
            }

        } else {
            telemetry.addLine("AUTO-AIM: No target found");
        }
    }


    @Override
    public void stop() {
        // Stop all active mechanisms
        if (shooterMotor != null) shooterMotor.setPower(0);
        if (shooterMotor2 != null) shooterMotor2.setPower(0);
        if (intakeMotor != null) intakeMotor.setPower(0);
        if (transferServo2 != null) transferServo2.setPosition(TRANSFER_DOWN_POS);
    }
}