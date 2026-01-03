


package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
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
@TeleOp(name = "Red-AutoAim-Teleop")
public class RedTeleop extends OpMode {

    // --- Pedro Pathing V2 Variables ---
    private Follower follower;
    private final double x = 80;
    private final double y = 8;
    private final Pose startPose = new Pose(x,y,0);

    private final double xf = 140;
    private final double yf = 144;
    double dx;
    double dy;
    // --- Turret System Variables (CR Simulation) ---
    private CRServo turretServo1;
    private CRServo turretServo2;
    private double turretMovePower = 0.2;
    private double kP = 28, kI = 0.05, kD = 2, kF = 0;
    private PIDController turretController;
    private double error = 0, integralSum =0, derivative =0;
    private DcMotorEx turretMoveMotor;
    double headingNeed = 0;
    double degrees = 0;
    double[] stepSizes = {10.0 , 1.0 , 0.1 , 0.01, 0.001};
    int stepIndex = 2;
    private boolean autoTurretEnabled = true;   // start ON (change if you want)
    private boolean x2_was_pressed = false;     // toggle button memory

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


    public double closeZoneTargetVelocity = 1850;
    public double farZoneTargetVelocity = 2200;
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
    private static final double INTAKE_BURST_DURATION = 0.200;
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




        try {
            // turretServo1 = hardwareMap.get(CRServo.class, "turretServo1");
            // turretServo2 = hardwareMap.get(CRServo.class, "turretServo2");
            // turretServo1.setPower(0);
            // turretServo2.setPower(0);
            // turretServo1.setDirection(DcMotorSimple.Direction.REVERSE);
            // turretServo2.setDirection(DcMotorSimple.Direction.REVERSE);
            turretMoveMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
            turretMoveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            turretMoveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMoveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turretMoveMotor.setTargetPosition(0);
            // Create FTCLib PIDF controller
            turretController = new PIDController(kP, kI, kD);
            turretController.setTolerance(0.5,1); // 2 degree tolerance
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
        follower = Constants.createFollower(hardwareMap);
        if (follower != null) {
            follower.setStartingPose(startPose);
            follower.update();
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
        boolean x2Pressed = gamepad2.x;

        if (x2Pressed && !x2_was_pressed) {
            autoTurretEnabled = !autoTurretEnabled;
        }

        x2_was_pressed = x2Pressed;


// --- TURRET ACTION LOGIC  ---
        if (autoTurretEnabled) {
            handleAutoTurretControl();   // auto-correct aiming
        } else {
            handleTurretControl();       // manual joystick control
        }

// Apply the calculated 'power' position to the CR servos
        //turretServo1.setPower(-turretMovePower);
        //turretServo2.setPower(-turretMovePower);

        turretMoveMotor.setVelocity(turretMovePower);

// --- MECHANISM CONTROL ---
        handleShooterControl();     // Gamepad 2 B (Toggle)
        handleIntakeControl();      // Gamepad 1 & 2 Bumpers (Manual)
        handleTransferControl();    // Gamepad 2 Dpad Up (Timed Cycle)

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
        telemetry.addData("PoseX: ", dx);
        telemetry.addData("PoseY: ", dy);
        telemetry.addData("PoseX: ", follower.getPose().getX());
        telemetry.addData("PoseY: ", follower.getPose().getY());
        telemetry.addData("Heading: ", follower.getHeading());
        telemetry.addData("EncoderTicks: ", turretMoveMotor.getCurrentPosition());
        telemetry.addData("Atan2: ", Math.atan2(dy,dx)*(180/Math.PI));
        telemetry.addData("shooter: ", shooterMotor.getVelocity());
        telemetry.addData("shooter2: ", shooterMotor2.getVelocity());

        telemetry.update();
    }

    //
//  SHOOTER CONTROL (GamePad2 B - TOGGLE)
//
    private void handleShooterControl() {
        if (shooterMotor == null || shooterMotor2 == null || StopperServo == null) return;

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

        // ---- Shooter velocity ----
        double targetVelocity = 0.0;
        if (shooterActive) {
            targetVelocity = (shooterMode == ShooterMode.CLOSE)
                    ? closeZoneTargetVelocity
                    : farZoneTargetVelocity;
        }

        shooterMotor.setVelocity(targetVelocity);
        shooterMotor2.setVelocity(targetVelocity);

        // ---- STOPPER LOGIC (NEW) ----
        if (shooterActive) {
            StopperServo.setPosition(STOPPER_UP_POS);     // open gate
        } else {
            StopperServo.setPosition(STOPPER_DOWN_POS);   // close gate
        }
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
        turretMovePower = (joystickInput*500);
    }



    private void handleAutoTurretControl(){
        dx = xf-follower.getPose().getX();
        dy = (yf)-(follower.getPose().getY());
        headingNeed = (Math.atan2(dy, dx) - follower.getHeading()) * 180/Math.PI;
        degrees = getAngle(turretMoveMotor.getCurrentPosition());
        error = headingNeed - degrees;
        integralSum += (error * timer.seconds());
        derivative = (error - lastError);
        // kF compensates for gravity or friction at target velocity
        //double feedforward = kF * Math.signum(error); // Direction-based
        turretMovePower = (kP * error) + (kI * integralSum) + (kD * derivative);
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

