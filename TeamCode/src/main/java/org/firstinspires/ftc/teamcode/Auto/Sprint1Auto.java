package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.nio.file.Paths;

@Autonomous(name = "Sprint1", group = "Examples")
public class Sprint1Auto extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState{
        DRIVE_STARTPOSE,
        SHOOT_PRELOAD
    }
    PathState pathState;
    private final Pose startPose = new Pose(66, 8, Math.toRadians(90));
    private final Pose shootPose = new Pose(66, 18, Math.toRadians(90));
    private PathChain driveToPreLoadShoot;

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

    }
    @Override
    public void init()
    {
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
    }

}
