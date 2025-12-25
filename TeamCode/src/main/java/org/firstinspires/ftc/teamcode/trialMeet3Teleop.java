/**



package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "ExampleDecTeleop-FieldCentric")
public class trialMeet3Teleop extends OpMode {

    // --- Pedro Pathing V2 Variables ---
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);

    // --- Turret System Variables (CR Simulation) ---
    private Servo turretServo1;
    private Servo turretServo2;
    private double turretMovePower = 0.5;

    // --- Shooter Variables ---
    private DcMotor shooterMotor;
    private DcMotor shooterMotor2;
    private static final double SHOOTER_MAX_POWER = -2.0; // Corrected to max valid power
    private boolean shooterActive = false;

    // --- Intake Variables ---
    private DcMotor intakeMotor;
    private static final double INTAKE_POWER = -1.0;

    // --- Toggle variables ---
    private boolean b_was_pressed = false; // For shooter toggle

    // --- Transfer Servo Variables ---
    private Servo transferServo;
    private static final double TRANSFER_DOWN_POS = 0.0; // Resting position
    private static final double TRANSFER_UP_POS = 0.7;   // Eject/Pushing position

    // State Machine Variables for Transfer/Indexing Cycle
    private enum TransferState {
        REST,
        PUSHING,
        RETURNING,
        INTAKE_BURST
    }
    private TransferState currentTransferState = TransferState.REST;
    private double pushStartTime = 0.0;
    private static final double PUSH_DURATION_SECONDS = 0.300; // 300ms for transfer push
    private double burstStartTime = 0.0;
    private static final double INTAKE_BURST_DURATION = 0.200; // 200ms for intake indexing
    private boolean dpadUp_was_pressed = false; // For single-click detection


    @Override
    public void init() {
// --- PEDRO PATHING V2 INITIALIZATION ---
        follower = Constants.createFollower(hardwareMap);
        if (follower != null) {
            follower.setStartingPose(startPose);
            follower.update();
        }

// --- HARDWARE INITIALIZATION ---

// Initialize Turret Servos (CR Servos)
        try {
            turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
            turretServo2 = hardwareMap.get(Servo.class, "turretServo2");
            turretServo1.setPosition(turretMovePower);
            turretServo2.setPosition(turretMovePower);
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find turret servos.");
        }

// Initialize Transfer Servo
        try {
            transferServo = hardwareMap.get(Servo.class, "transferServo"); // Check this config name!
            transferServo.setPosition(TRANSFER_DOWN_POS);
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find transfer servo.");
        }

// Initialize Motors
        try {
            shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
            shooterMotor2 = hardwareMap.get(DcMotor.class, "shooterMotor2");
            intakeMotor = hardwareMap.get(DcMotor.class, "Intake");

            // Set modes once for all
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        } catch (Exception e) {
            telemetry.addData("Error", "Could not find one or more motors (Shooter/Intake).");
        }

        telemetry.update();
    }

    @Override
    public void start() {
        if (follower != null) follower.startTeleopDrive();
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

// --- TURRET ACTION LOGIC (Continuous Rotation Servo) ---
        handleTurretControl();

// Apply the calculated 'power' position to the CR servos
        if (turretServo1 != null && turretServo2 != null) {
            turretServo1.setPosition(turretMovePower);
            turretServo2.setPosition(turretMovePower);
        }

// --- MECHANISM CONTROL ---
        handleShooterControl();     // Gamepad 2 B (Toggle)
        handleIntakeControl();      // Gamepad 1 & 2 Bumpers (Manual)
        handleTransferControl();    // Gamepad 2 Dpad Up (Timed Cycle)

// --- TELEMETRY ---
        telemetry.addData("Drive Mode", "FIELD CENTRIC");
        telemetry.addData("Turret Power Pos", "%.3f", turretMovePower);
        telemetry.addData("Shooter Active (B)", shooterActive);
        telemetry.addData("Intake Control", "R/L Bumpers 1 & 2");
        telemetry.addData("Transfer State", currentTransferState.toString());
        telemetry.addData("Robot Pose", follower != null ? follower.getPose() : "N/A");

        telemetry.update();
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
// ---------- INTAKE CONTROL (GamePad1 & GamePad2 Bumpers) - MODIFIED ----------
//
    private void handleIntakeControl() {
        if (intakeMotor == null) return;

// Manual control will set power based on bumper press.
// It relies on the transfer state machine to turn the motor off when resting.
        boolean intakeForward = gamepad1.right_bumper || gamepad2.right_bumper;
        boolean intakeReverse = gamepad1.left_bumper || gamepad2.left_bumper;

        if (intakeForward) {
            // Intake forward (Suck in)
            intakeMotor.setPower(INTAKE_POWER);
        } else if (intakeReverse) {
            // Intake reverse (Outtake/Spit out)
            intakeMotor.setPower(0.25);
        }
    }


    //
// ---------- MANUAL TURRET CONTROL (GamePad2 Left Stick X - CR SERVO) ----------
//
    private void handleTurretControl() {
        double joystickInput = gamepad2.left_stick_x;
        // Map joystick (-1.0 to 1.0) to CR servo range (0.0 to 1.0), where 0.5 is stop.
        turretMovePower = (joystickInput + 1.0) / 2.0;
        turretMovePower = Range.clip(turretMovePower, 0.0, 1.0);
    }


    //
// ---------- TRANSFER CONTROL (GamePad2 Dpad Up - TIMED CYCLE with Intake) ----------
//
    private void handleTransferControl() {
        if (transferServo == null) return;
        if (intakeMotor == null) return; // Added check for intake motor since it's used here

        boolean dpad_up_is_pressed = gamepad2.dpad_up;

        switch (currentTransferState) {
            case REST:
                // Intake Motor Control: Turn motor off UNLESS manual bumper control is active
                if (!gamepad1.right_bumper && !gamepad2.right_bumper &&
                        !gamepad1.left_bumper && !gamepad2.left_bumper) {
                    intakeMotor.setPower(0);
                }

                // Check for single-click input to start cycle
                if (dpad_up_is_pressed && !dpadUp_was_pressed) {
                    // Start PUSHING action
                    transferServo.setPosition(TRANSFER_UP_POS);
                    pushStartTime = getRuntime();
                    currentTransferState = TransferState.PUSHING;
                }
                break;

            case PUSHING:
                // Wait for the push duration (0.300s) to expire
                if (getRuntime() >= pushStartTime + PUSH_DURATION_SECONDS) {
                    // Time is up, immediately move transfer servo down
                    transferServo.setPosition(TRANSFER_DOWN_POS);
                    currentTransferState = TransferState.RETURNING;
                }
                break;

            case RETURNING:
                // Servo is moving down. Immediately start the Intake Burst timer.
                burstStartTime = getRuntime(); // Record the start of the burst
                currentTransferState = TransferState.INTAKE_BURST;
                break;

            case INTAKE_BURST:
                // Run the intake motor at max power for 0.200s
                intakeMotor.setPower(INTAKE_POWER);

                // Wait for the burst duration to expire
                if (getRuntime() >= burstStartTime + INTAKE_BURST_DURATION) {
                    // Time is up, stop the intake and return to REST
                    intakeMotor.setPower(0);
                    currentTransferState = TransferState.REST;
                }
                break;
        }

        // --- Update the single-click tracker ---
        dpadUp_was_pressed = dpad_up_is_pressed;
    }

    @Override
    public void stop() {
// Stop all active mechanisms
        if (shooterMotor != null) shooterMotor.setPower(0);
        if (shooterMotor2 != null) shooterMotor2.setPower(0);
        if (intakeMotor != null) intakeMotor.setPower(0);
        if (turretServo1 != null) turretServo1.setPosition(0.5);
        if (turretServo2 != null) turretServo2.setPosition(0.5);
        if (transferServo != null) transferServo.setPosition(TRANSFER_DOWN_POS);
    }
}

 */



package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "ExampleDecTeleop-FieldCentric")
public class trialMeet3Teleop extends OpMode {

    // --- Pedro Pathing V2 Variables ---
    private Follower follower;
    private final double x = 8;
    private final double y = 82;
    private final Pose startPose = new Pose(x,y,0);

    private final double xf = 133;
    private final double yf = 15;
    // --- Turret System Variables (CR Simulation) ---
    private CRServo turretServo1;
    private CRServo turretServo2;
    private double turretMovePower = 0.2;
    private double kP = 28, kI = 0.05, kD = 2, kF = 7;
    private double error = 0, integralSum =0, derivative =0;
    private DcMotorEx turretMoveMotor;
    double headingNeed = 0;
    double degrees = 0;
    double[] stepSizes = {10.0 , 1.0 , 0.1 , 0.01, 0.001};
    int stepIndex = 2;

    // --- Shooter Variables ---
    private DcMotorEx shooterMotor;
    private DcMotorEx shooterMotor2;
    private boolean a_was_pressed = false;
    private boolean b_was_pressed = false;

    private enum ShooterMode {
        CLOSE,
        FAR
    }

    private ShooterMode shooterMode = ShooterMode.CLOSE;
    private boolean shooterActive = false;


    public double closeZoneTargetVelocity = 1800;
    public double farZoneTargetVelocity = 2100;
    private static final double SHOOTER_MAX_POWER = 1.0;
    //private boolean shooterActive = false;

    // --- Intake Variables ---
    private DcMotor intakeMotor;
    private static final double INTAKE_POWER = -1.0;

    // --- Toggle variables ---
    //private boolean b_was_pressed = false; // For shooter toggle

    // --- Transfer Servo 1 (Cycling Mechanism) Variables ---
    private Servo transferServo;
    private static final double TRANSFER_DOWN_POS = 0.5;
    private static final double TRANSFER_UP_POS = 0.85;


    private enum TransferState {
        REST,
        PUSHING,
        RETURNING,
        INTAKE_BURST
    }
    private TransferState currentTransferState = TransferState.REST;
    private double pushStartTime = 0.0;
    private static final double PUSH_DURATION_SECONDS = 0.300;
    private double burstStartTime = 0.0;
    private static final double INTAKE_BURST_DURATION = 0.250;
    private boolean dpadUp2_was_pressed = false;

    // Transfer Servo 2 (Stopper/Gate Mechanism)
    private Servo StopperServo;
    private static final double STOPPER_DOWN_POS = 0.267;
    private static final double STOPPER_UP_POS = 0.05;
    private boolean stopper_is_up = false;
    private boolean dpadUp1_was_pressed = false;

    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        if (follower != null) {
            follower.setStartingPose(startPose);
            follower.update();
        }



        try {
           // turretServo1 = hardwareMap.get(CRServo.class, "turretServo1");
           // turretServo2 = hardwareMap.get(CRServo.class, "turretServo2");
           // turretServo1.setPower(0);
           // turretServo2.setPower(0);
           // turretServo1.setDirection(DcMotorSimple.Direction.REVERSE);
           // turretServo2.setDirection(DcMotorSimple.Direction.REVERSE);
            turretMoveMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
            turretMoveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            PIDFCoefficients pidfCoefficientsTurret = new PIDFCoefficients(kP,kI,kD,kF);
            turretMoveMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficientsTurret);
            turretMoveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMoveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turretMoveMotor.setTargetPosition(0);
            //turretMoveMotor.setPower(0); // see if this is okay
           // turretMoveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            PIDFCoefficients pidfCoefficentsShooter = new PIDFCoefficients(0.4,0,0,13.2);
            shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficentsShooter);
            shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficentsShooter);
            shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            shooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);


        } catch (Exception e){
            telemetry.addData("Error", "Could not find turret servos.");
        }

// Initialize Transfer Servo 1 (Cycling)
        try {
            transferServo = hardwareMap.get(Servo.class, "TransferServo");
            transferServo.setPosition(TRANSFER_DOWN_POS);
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find transfer servo 1.");
        }

// Initialize Transfer Servo 2 (Stopper) <-- NEW INITIALIZATION
        try {
            StopperServo = hardwareMap.get(Servo.class, "StopperServo");
            StopperServo.setPosition(STOPPER_DOWN_POS); // Start at 0.0 (Outwards/Open)
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find Stopper Servo (Stopper).");
        }


// Initialize Motors
        try {
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
            shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
            intakeMotor = hardwareMap.get(DcMotor.class, "Intake");

            // Set modes once for all
            PIDFCoefficients pidfCoefficentsShooter = new PIDFCoefficients(0.4,0,0,13.2);
            shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficentsShooter);
            shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficentsShooter);
            shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            shooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        } catch (Exception e) {
            telemetry.addData("Error", "Could not find one or more motors (Shooter/Intake).");
        }

        telemetry.update();
    }

    @Override
    public void start() {
        if (follower != null) follower.startTeleopDrive();
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
            if(gamepad1.bWasPressed()){
                stepIndex = (stepIndex +1)% stepSizes.length;
            }

            if(gamepad1.dpadLeftWasPressed())
            {
                kD -= stepSizes[stepIndex];
            }
            if(gamepad1.dpadRightWasPressed())
            {
                kD+= stepSizes[stepIndex];
            }
            if(gamepad1.dpadUpWasPressed())
            {
                kP-= stepSizes[stepIndex];
            }
            if(gamepad1.dpadDownWasPressed())
            {
                kP+= stepSizes[stepIndex];
            }
            if(gamepad2.dpadLeftWasPressed())
            {
                kF -= stepSizes[stepIndex];
            }
            if(gamepad2.dpadRightWasPressed())
            {
                kF+= stepSizes[stepIndex];
            }
            if(gamepad2.dpadUpWasPressed())
            {
                kI-= stepSizes[stepIndex];
            }
            if(gamepad2.dpadDownWasPressed())
            {
                kI+= stepSizes[stepIndex];
            }
            PIDFCoefficients pidfCoefficientsTurret = new PIDFCoefficients(kP,kI,kD,kF);
            turretMoveMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficientsTurret);


        }


// --- TURRET ACTION LOGIC (CR Servo) ---
        handleTurretControl();
        handleAutoTurretControl();

// Apply the calculated 'power' position to the CR servos
            //turretServo1.setPower(-turretMovePower);
            //turretServo2.setPower(-turretMovePower);

       turretMoveMotor.setVelocity(turretMovePower);

// --- MECHANISM CONTROL ---
        handleShooterControl();     // Gamepad 2 B (Toggle)
        handleIntakeControl();      // Gamepad 1 & 2 Bumpers (Manual)
        handleTransferControl();    // Gamepad 2 Dpad Up (Timed Cycle)
        handleStopperControl();     // Gamepad 1 Dpad Up (Toggle Stopper)

// --- TELEMETRY ---
        telemetry.addData("Drive Mode", "FIELD CENTRIC");
        telemetry.addData("Stopper Pos (G1 Dpad Up)", StopperServo != null ? StopperServo.getPosition() : "N/A");
        telemetry.addData("Transfer State (G2 Dpad Up)", currentTransferState.toString());
        telemetry.addData("Shooter Active (G2 B)", shooterActive);
        telemetry.addData("Robot Pose", follower != null ? follower.getPose() : "N/A");
        telemetry.addData("Transfer Position", transferServo.getPosition());
        telemetry.addData("Current", "%.2f°", degrees);
        telemetry.addData("Target", "%.2f°", headingNeed);
        telemetry.addData("Error", "%.2f°", error);
        telemetry.addData("Power", "%.2f", turretMovePower);
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", kP );
        telemetry.addData("Tuning D", "%.4f (D-Pad L/R)", kD );
        telemetry.addData("Tuning I", "%.4f (D-Pad U/D)", kI );
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", kF );
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
        telemetry.addData("PoseX: ", follower.getPose().getX());
        telemetry.addData("PoseY: ", follower.getPose().getY());
        telemetry.addData("Heading: ", follower.getHeading());
        telemetry.addData("EncoderTicks: ", turretMoveMotor.getCurrentPosition());

        telemetry.update();
    }

    //
//  SHOOTER CONTROL (GamePad2 B - TOGGLE)
//
    private void handleShooterControl() {
        if (shooterMotor == null && shooterMotor2 == null) return;

        boolean aPressed = gamepad2.a;
        boolean bPressed = gamepad2.b;

        // ---- B toggles CLOSE zone ----
        if (bPressed && !b_was_pressed) {
            if (shooterActive && shooterMode == ShooterMode.CLOSE) {
                shooterActive = false; // turn off
            } else {
                shooterActive = true;
                shooterMode = ShooterMode.CLOSE;
            }
        }

        // ---- A toggles FAR zone ----
        if (aPressed && !a_was_pressed) {
            if (shooterActive && shooterMode == ShooterMode.FAR) {
                shooterActive = false; // turn off
            } else {
                shooterActive = true;
                shooterMode = ShooterMode.FAR;
            }
        }

        a_was_pressed = aPressed;
        b_was_pressed = bPressed;

        double targetVelocity = 0.0;

        if (shooterActive) {
            if (shooterMode == ShooterMode.CLOSE) {
                targetVelocity = closeZoneTargetVelocity;
            } else if (shooterMode == ShooterMode.FAR) {
                targetVelocity = farZoneTargetVelocity;
            }
        }

        shooterMotor.setVelocity(targetVelocity);
        shooterMotor2.setVelocity(targetVelocity);
    }




    //
// INTAKE CONTROL (GamePad1 & GamePad2 Bumpers)
//
    private void handleIntakeControl() {
        if (intakeMotor == null) return;

        boolean intakeForward = gamepad1.right_bumper || gamepad2.right_bumper;
        boolean intakeReverse = gamepad1.left_bumper || gamepad2.left_bumper;

        if (intakeForward) {
            intakeMotor.setPower(INTAKE_POWER);
        } else if (intakeReverse) {
            intakeMotor.setPower(0.25);
        }
    }



// MANUAL TURRET CONTROL (GamePad2 Left Stick X - CR SERVO)

    private void handleTurretControl() {
        double joystickInput = gamepad2.left_stick_x;
        turretMovePower = (joystickInput*0.7);
    }

    private void handleAutoTurretControl(){
        double dx = xf-follower.getPose().getX();
        double dy = (yf-follower.getPose().getY());
        headingNeed = (Math.atan2(dx,dy) - follower.getHeading())*(180/Math.PI);
        degrees = getAngle(turretMoveMotor.getCurrentPosition());
        error = headingNeed - degrees;
        integralSum += (error * timer.seconds());
        derivative = (error - lastError);

        turretMovePower = (kP * error) + (kI*integralSum) + (kD * derivative);
        lastError = error;
        timer.reset();
    }

    public double getAngle(double encoderPosition)
    {
        double kV = 963/180;
        return encoderPosition/kV;
    }


// TRANSFER CONTROL (GamePad2 Dpad Up - TIMED CYCLE with Intake)

    private void handleTransferControl() {
        if (transferServo == null || intakeMotor == null) return;

        boolean dpad_up_is_pressed = gamepad2.y;

        switch (currentTransferState) {
            case REST:

                if (!gamepad1.right_bumper && !gamepad2.right_bumper &&
                        !gamepad1.left_bumper && !gamepad2.left_bumper) {
                    intakeMotor.setPower(0);
                }


                if (dpad_up_is_pressed && !dpadUp2_was_pressed) {
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

        dpadUp2_was_pressed = dpad_up_is_pressed;
    }



//  STOPPER CONTROL (GamePad1 Dpad Up - TOGGLE)

    private void handleStopperControl() {
        if (StopperServo == null) return;

        boolean dpad_up_is_pressed = gamepad1.x; // Note: Uses Gamepad 1

        if (dpad_up_is_pressed && !dpadUp1_was_pressed) {
            // Toggle the state
            stopper_is_up = !stopper_is_up;

            // Set the servo position based on the new state
            if (stopper_is_up) {
                StopperServo.setPosition(STOPPER_UP_POS); // 0.5 (Closed/Up)
            } else {
                StopperServo.setPosition(STOPPER_DOWN_POS); // 0.0 (Open/Down)
            }
        }

        // Update the button tracker
        dpadUp1_was_pressed = dpad_up_is_pressed;
    }

    @Override
    public void stop() {
// Stop all active mechanisms
        if (shooterMotor != null) shooterMotor.setPower(0);
        if (shooterMotor2 != null) shooterMotor2.setPower(0);
        if (intakeMotor != null) intakeMotor.setPower(0);
        if (turretServo1 != null) turretServo1.setPower(0);
        if (turretServo2 != null) turretServo2.setPower(0);
        if (transferServo != null) transferServo.setPosition(TRANSFER_DOWN_POS);
        if (StopperServo != null) StopperServo.setPosition(STOPPER_DOWN_POS); // Stop the stopper
    }
}