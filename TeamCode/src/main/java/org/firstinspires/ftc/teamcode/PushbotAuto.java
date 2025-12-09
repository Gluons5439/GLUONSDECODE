package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "PushbotAuto", group = "Examples")
public class PushbotAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Create follower
        Follower follower = Constants.createFollower(hardwareMap);

        // Set starting pose (optional, defaults to 0,0,0)
        follower.setStartingPose(new Pose(0, 0, 0));

        // Initial update
        follower.update();

        waitForStart();
        if (isStopRequested()) return;

        // Build a simple straight path
        Path path = new Path(new BezierLine(follower::getPose, new Pose(24, 0, 0)));

        // Follow the path
        follower.followPath(path);

        // Wait until the follower is done
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("pose", follower.getPose());
            telemetry.update();
        }

        // Done - optionally stop motors or switch to teleop control
    }
}
