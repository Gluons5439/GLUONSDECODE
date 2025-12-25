package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.BezierLine;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Forward")
public class Forward extends LinearOpMode {

    private Follower follower;

    @Override
    public void runOpMode() {

        // Initialize follower
        follower = Constants.createFollower(hardwareMap);

        // Set starting pose
        follower.setStartingPose(new Pose(0, 0, 0)); // x=0, y=0, heading=0 radians

        // Build a simple straight path forward (for example, 24 inches forward on x-axis)
        PathChain forwardPath = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(0, 0), new Pose(24, 0))) // Move 24 units forward
                .build();

        waitForStart();

        if (isStopRequested()) return;

        // Follow the path synchronously
        follower.followPath(forwardPath);

        // Optionally, stop the robot after path completion
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0, 0, 0, true);
    }
}
