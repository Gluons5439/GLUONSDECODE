package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Auto.Mechanisms.FlyWheelLogic;
import org.firstinspires.ftc.teamcode.Auto.Mechanisms.IntakeLogic;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Sprint1", group = "Examples")
public class Sprint1Auto extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    DcMotorEx turretMoveMotor;
    private IntakeLogic intake;

    private Servo StopperServo;
    private static final double STOPPER_DOWN_POS = 0.267;
    private static final double STOPPER_UP_POS = 0.05;


    public enum PathState{
        DRIVE_STARTPOSE,
        SHOOT_PRELOAD,
        DRIVE_BETWEEN,
        DRIVE_FRONT_COLLECT,
        COLLECT,
        SHOOT_SET_1
    }
    PathState pathState;
    private final Pose startPose = new Pose(66, 8, Math.toRadians(90));
    private final Pose shootPose = new Pose(66, 14, Math.toRadians(122));

    private final Pose frontCollect1 = new Pose(42,35.5, Math.toRadians(180));
    private final Pose collect1 = new Pose(20,35.5, Math.toRadians(180));
    private PathChain driveToPreLoadShoot;
    private PathChain driveToFrontCollect;
    private PathChain collectBallSet1;

    private PathChain driveToCollect1Shoot;
    private FlyWheelLogic shooter;
    private boolean shotsTriggered = false;
    private boolean pathStarted = false;


    public void buildPaths() {
        driveToPreLoadShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        driveToFrontCollect = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,frontCollect1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), frontCollect1.getHeading())
                .build();
        collectBallSet1 = follower.pathBuilder()
                .addPath(new BezierLine(frontCollect1,collect1))
                .setLinearHeadingInterpolation(frontCollect1.getHeading(),collect1.getHeading())
                .build();
        driveToCollect1Shoot = follower.pathBuilder()
                .addPath(new BezierLine(collect1,shootPose))
                .setLinearHeadingInterpolation(collect1.getHeading(), shootPose.getHeading())
                .build();

    }

    public void statePathUpdate() {
        switch (pathState) {

            case DRIVE_STARTPOSE:
                if (!pathStarted) {
                    follower.followPath(driveToPreLoadShoot, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                if (!shotsTriggered) {
                    shooter.fireShots(3);
                    shotsTriggered = true;
                }
                else if (!shooter.isBusy()) {
                    StopperServo.setPosition(STOPPER_DOWN_POS);
                    setPathState(PathState.DRIVE_FRONT_COLLECT);
                }
                break;

            case DRIVE_FRONT_COLLECT:
                if (!pathStarted) {
                    follower.followPath(driveToFrontCollect, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.COLLECT);
                }
                break;

            case COLLECT:
                intake.setZaPower(0.9);
                if (!pathStarted) {
                    follower.followPath(collectBallSet1, 0.5, true);
                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_SET_1);
                }
                break;

            case SHOOT_SET_1:

                if (!pathStarted) {
                    follower.followPath(driveToCollect1Shoot, true);
                    StopperServo.setPosition(STOPPER_UP_POS);
                    shooter.fireShots(3);
                    shotsTriggered = true;
                    pathStarted = true;
                }

                if (!follower.isBusy() && shotsTriggered && !shooter.isBusy()) {
                    telemetry.addLine("Done all Paths!");
                }
                break;
        }
    }



    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        shotsTriggered = false;
        pathStarted = false; // CRITICAL
    }

    @Override
    public void init()
    {
        StopperServo = hardwareMap.get(Servo.class, "StopperServo");
        StopperServo.setPosition(STOPPER_UP_POS);

        turretMoveMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMoveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMoveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMoveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = new IntakeLogic();
        intake.init(hardwareMap);

        turretMoveMotor.setPower(0);
        pathState = PathState.DRIVE_STARTPOSE;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        shooter = new FlyWheelLogic();
        shooter.init(hardwareMap);


        buildPaths();
        follower.setPose(startPose);
    }
    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }
    @Override
    public void loop()
    {
        follower.update();
        statePathUpdate();
        shooter.update();

        telemetry.addData("Path state: ", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());


    }

}
