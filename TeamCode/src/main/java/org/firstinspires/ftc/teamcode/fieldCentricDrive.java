package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// Assumes Constants.java exists in the same package to configure the Follower
// The @Configurable annotation is included to match the V2 example structure
@Configurable
@TeleOp(name = "Example Field-Centric TeleOp V2", group = "Examples")
public class fieldCentricDrive extends OpMode {

    // --- Pedro Pathing V2 Variables ---
    private Follower follower;
    // Starting pose is used to set the initial position for field-centric orientation
    private final Pose startPose = new Pose(0, 0, 0);

    // --- Turret System Variables ---
    private Servo turretServo1;
    private Servo turretServo2;

    private double turretTargetPosition = 0.5;

    // --- Shooter Variables ---
    private DcMotor shooterMotor;
    private static final double SHOOTER_MAX_POWER = 1.0;

    // --- Vision/Limelight Variables ---
    private Limelight3A limelight;

    // --- Auto-Aim PID Constants (for proportional control) ---
    private static final double AIM_KP = 0.0008;
    private static final double AIM_DEADZONE = 1.5; // Degrees of error allowed before correction stops
    private static final double MAX_CORRECTION = 0.001; // Max servo movement per loop

    // --- Toggle variables for Turret/Auto-Aim state ---
    private boolean turretEnabled = false;  // turret starts OFF
    private boolean b_was_pressed = false;

    private boolean isAutoAimActive = false;
    private boolean a_was_pressed = false;

    @Override
    public void init() {
        // --- PEDRO PATHING V2 INITIALIZATION ---
        // This relies on the static method in Constants.java to initialize the Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update(); // Initial update to apply starting pose

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

        // Initialize Shooter Motor
        try {
            shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("Shooter", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Error", "Shooter not found.");
        }

        // Initialize Limelight
        try {
            limelight = hardwareMap.get(Limelight3A.class, "Limelight");
            limelight.pipelineSwitch(0);
            telemetry.addData("Limelight", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Error", "Limelight not found.");
        }

        telemetry.update();
    }

    @Override
    public void start() {
        // Start the TeleOp drive functionality in the follower
        follower.startTeleopDrive();
        if (limelight != null) limelight.start();
    }

    @Override
    public void loop() {
        // --- PEDRO PATHING V2 DRIVING ---
        // setTeleOpDrive is the V2 equivalent of setTeleOpMovementVectors
        // The last parameter (false) enforces FIELD-CENTRIC driving
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y, // Translational power (Forward/Backward)
                -gamepad1.left_stick_x, // Strafe power (Left/Right)
                -gamepad1.right_stick_x, // Rotational power (Turning)
                false  // FIELD CENTRIC (Rotation is relative to the field's starting orientation)
        );
        // Update the follower to execute the drive commands and localization
        follower.update();

        // --- TURRET ENABLE/DISABLE TOGGLE (GamePad2 B) ---
        boolean b_is_pressed = gamepad2.b;
        if (b_is_pressed && !b_was_pressed) {
            turretEnabled = !turretEnabled;

            // Auto-Aim turns off when turret disabled
            if (!turretEnabled) {
                isAutoAimActive = false;
            }
        }
        b_was_pressed = b_is_pressed;

        // --- AUTO-AIM TOGGLE (GamePad2 A) ---
        boolean a_is_pressed = gamepad2.a;
        if (turretEnabled && a_is_pressed && !a_was_pressed) {
            isAutoAimActive = !isAutoAimActive;
        }
        a_was_pressed = a_is_pressed;

        // --- TURRET ACTION LOGIC ---
        if (turretEnabled) {
            if (isAutoAimActive) {
                handleAutoAim();
            } else {
                handleTurretControl();
            }
        }

        // Clip the target position to servo limits (0.0 to 1.0)
        turretTargetPosition = Range.clip(turretTargetPosition, 0.0, 1.0);
        // Apply the calculated or manually set position to the servos
        if (turretServo1 != null && turretServo2 != null) {
            turretServo1.setPosition(turretTargetPosition);
            turretServo2.setPosition(turretTargetPosition);
        }

        // --- SHOOTER CONTROL ---
        handleShooterControl();

        // --- TELEMETRY ---
        telemetry.addData("Turret Enabled (B)", turretEnabled);
        telemetry.addData("Auto-Aim (A)", isAutoAimActive);
        telemetry.addData("Turret Position", turretTargetPosition);
        telemetry.addData("Shooter Power", shooterMotor != null ? shooterMotor.getPower() : 0);
        telemetry.addData("Robot Pose", follower.getPose());

        telemetry.update();
    }


    //
    // ---------- SHOOTER CONTROL (GamePad2 Right Bumper) ----------
    //
    private void handleShooterControl() {
        if (shooterMotor == null) return;

        if (gamepad2.right_bumper) {
            shooterMotor.setPower(SHOOTER_MAX_POWER);
        } else {
            shooterMotor.setPower(0.0);
        }
    }

    //
    // ---------- MANUAL TURRET CONTROL (GamePad2 Left Stick X) ----------
    //
    private void handleTurretControl() {
        // Map the joystick input (-1.0 to 1.0) to servo range (0.0 to 1.0)
        double turretMoveInput = gamepad2.left_stick_x;
        // This is a simple proportional map: Input -1 -> 0.0, Input 0 -> 0.5, Input 1 -> 1.0
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

        // Get the latest vision data from the Limelight
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // tx is the horizontal offset of the target from the center of the camera in degrees
            double tx = result.getTx();

            telemetry.addData("Target X (°)", tx);

            // Check if the error is outside the acceptable deadzone
            if (Math.abs(tx) > AIM_DEADZONE) {
                // Calculate proportional correction: error * Kp
                double correction = tx * AIM_KP;
                // Clip the correction to ensure we don't move the servo too quickly
                correction = Range.clip(correction, -MAX_CORRECTION, MAX_CORRECTION);

                // Apply the correction to the servo position.
                // Note: The sign is determined by the servo setup. If the turret moves the wrong way,
                // change -= correction to += correction, or invert AIM_KP.
                turretTargetPosition -= correction;

                telemetry.addLine("AUTO-AIM: Tracking...");
                telemetry.addData("Correction", correction);

            } else {
                telemetry.addLine("AUTO-AIM: Locked on!");
            }

        } else {
            telemetry.addLine("AUTO-AIM: No target found");
        }
    }


    @Override
    public void stop() {
        // Stop the shooter motor and the drive motors (handled by Follower)
        if (shooterMotor != null) shooterMotor.setPower(0);
    }
}