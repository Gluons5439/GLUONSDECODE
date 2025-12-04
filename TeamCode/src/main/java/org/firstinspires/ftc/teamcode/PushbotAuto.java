package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Pushbot Auto OpMode.
 * Follows multiple paths sequentially using Pedro Pathing.
 */
@Autonomous(name = "PushbotAuto")
public class PushbotAuto extends LinearOpMode {

    private Follower follower;

    public PathChain path1, path2, path3, path4, path5, path6, path7, path8;

    @Override
    public void runOpMode() {

        // Initialize the follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());

        // Build all paths
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.000, 8.000), new Pose(56.187, 35.337)))
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.187, 35.337), new Pose(13.428, 35.161)))
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(13.428, 35.161), new Pose(73.148, 76.329)))
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(73.148, 76.329), new Pose(48.412, 59.367)))
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(48.412, 59.367), new Pose(12.721, 59.367)))
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(12.721, 59.367), new Pose(63.607, 83.573)))
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(63.607, 83.573), new Pose(12.545, 83.573)))
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(12.545, 83.573), new Pose(63.784, 83.573)))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        // Follow all paths sequentially
        follower.followPath(path1);
        follower.followPath(path2);
        follower.followPath(path3);
        follower.followPath(path4);
        follower.followPath(path5);
        follower.followPath(path6);
        follower.followPath(path7);
        follower.followPath(path8);
    }
}
