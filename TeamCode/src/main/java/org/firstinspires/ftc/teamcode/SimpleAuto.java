package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Minimal PedroPathing v2 example autonomous.
 * Moves forward ~24 units (in whatever units your localizer uses; typically inches).
 */
/*
@Autonomous(name = "SimpleAuto - Pedro v2", group = "Examples")
public class SimpleAutoA extends LinearOpMode {
    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        // Create follower using the v2 factory in Constants
        follower = Constants.createFollower(hardwareMap);

        // Optionally set a starting pose (defaults to 0,0,0 if not set)
        follower.setStartingPose(new Pose(0, 0, 0));

        // One update call before starting (example docs do this)
        follower.update();

        waitForStart();
        if (isStopRequested()) return;

        // Build a simple straight path (BezierLine from current pose to 24 forward)
        PathChain chain = follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(24, 0, 0))))
                .build();

        // Tell the follower to follow the chain
        follower.followPathChain(chain);

        // Keep updating until follower reports it's no longer busy
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            // optional: telemetry to see progress
            telemetry.addData("pose", follower.getPose());
            telemetry.update();
        }

        // Done — optionally stop motors or switch to teleop drive
        // follower.startTeleopDrive(); // if you want to return control to teleop-style drive
    }
}
*/