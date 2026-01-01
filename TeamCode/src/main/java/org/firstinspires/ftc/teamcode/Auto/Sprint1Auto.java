package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Auto.Mechanisms.FlyWheelLogic;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Sprint1", group = "Examples")
public class Sprint1Auto extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private Servo StopperServo;
    private static final double STOPPER_DOWN_POS = 0.267;
    private static final double STOPPER_UP_POS = 0.05;


    public enum PathState{
        DRIVE_STARTPOSE,
        SHOOT_PRELOAD
    }
    PathState pathState;
    private final Pose startPose = new Pose(66, 8, Math.toRadians(90));
    private final Pose shootPose = new Pose(66, 18, Math.toRadians(90));
    private PathChain driveToPreLoadShoot;
    private FlyWheelLogic shooter = new FlyWheelLogic();
    private boolean shotsTriggered = false;

    public void buildPaths() {
        driveToPreLoadShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate()
    {
        switch (pathState){
            case DRIVE_STARTPOSE:
                follower.followPath(driveToPreLoadShoot,true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if(!follower.isBusy()){
                    telemetry.addLine("path 1 done");
                    if(!shotsTriggered)
                    {
                        shooter.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.isBusy())
                    {
                        telemetry.addLine("Done all Paths!");
                    }
                }
                break;
            default:
                telemetry.addLine("No state Commanded");
                break;
        }

    }
    public void setPathState(PathState newState)
    {
        pathState = newState;
        pathTimer.resetTimer();

        shotsTriggered =false;

    }
    @Override
    public void init()
    {
        StopperServo = hardwareMap.get(Servo.class, "StopperServo");
        StopperServo.setPosition(STOPPER_UP_POS);
        pathState = PathState.DRIVE_STARTPOSE;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

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
