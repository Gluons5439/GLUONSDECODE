package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "NewAutoTrial", group = "Examples")
public class NewAutoTrial extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Create follower
        Follower follower = Constants.createFollower(hardwareMap);

        // Set starting pose
        follower.setStartingPose(new Pose(56, 8, 0));

        // Initial update
        follower.update();

        waitForStart();
        if (isStopRequested()) return;

        // Create all paths
        Paths paths = new Paths(follower);

        CRServo transferServo1; // NEW CR Servo
        CRServo transferServo2; // NEW CR Servo

        // Shooter state variable
        boolean shooterState = false;
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        intakeMotor.setPower(-1);

        // First batch: Path1, Path2, Path3
        PathChain[] firstBatch = new PathChain[]{paths.Path1, paths.Path2, paths.Path3};
        for (PathChain path : firstBatch) {
            follower.followPath(path);
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                telemetry.addData("X", follower.getPose().getX());
                telemetry.addData("Y", follower.getPose().getY());
                telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
                telemetry.update();
            }
        }

        // Set shooterState to run motor
        DcMotor shooterMotor1 = hardwareMap.get(DcMotor.class, "shooterMotor");
        DcMotor shooterMotor2 = hardwareMap.get(DcMotor.class, "shooterMotor2");

        shooterMotor1.setPower(-1);
        shooterMotor2.setPower(-1);
        sleep(3000);
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);

        // Second batch: Path4, Path5, Path6
        PathChain[] secondBatch = new PathChain[]{paths.Path4, paths.Path5, paths.Path6};
        for (PathChain path : secondBatch) {
            follower.followPath(path);
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                telemetry.addData("X", follower.getPose().getX());
                telemetry.addData("Y", follower.getPose().getY());
                telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
                telemetry.update();
            }
        }

        // Another motor action
        shooterMotor1.setPower(-1);
        shooterMotor2.setPower(-1);
        sleep(3000);
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);

        // Final batch: Path7, Path8, Path9
        PathChain[] finalBatch = new PathChain[]{paths.Path7, paths.Path8};
        for (PathChain path : finalBatch) {
            follower.followPath(path);
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                telemetry.addData("X", follower.getPose().getX());
                telemetry.addData("Y", follower.getPose().getY());
                telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
                telemetry.update();
            }
        }

        shooterMotor1.setPower(-1);
        shooterMotor2.setPower(-1);
        sleep(3000);
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
        intakeMotor.setPower(0);
        PathChain[] thirdbatch = new PathChain[]{paths.Path9};
        for (PathChain path : thirdbatch) {
            follower.followPath(path);
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                telemetry.addData("X", follower.getPose().getX());
                telemetry.addData("Y", follower.getPose().getY());
                telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
                telemetry.update();
            }
        }
    }

    // Nested class for your paths
    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(56.000, 8.000), new Pose(56.000, 36.000)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(56.000, 36.000), new Pose(13.561379165237916, 35.13742204343262)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(13.561379165237916, 35.13742204343262), new Pose(72.54325094640056, 85.371694887308)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(72.54325094640056, 85.371694887308), new Pose(55.10318550148585, 60.3247923866326)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(55.10318550148585, 60.3247923866326), new Pose(12.616217555895748, 59.397129331052035)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(12.616217555895748, 59.397129331052035), new Pose(72.72878355751666, 85.18616227619187)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(72.72878355751666, 85.18616227619187), new Pose(12.245152333663519, 83.51636877614685)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(12.245152333663519, 83.51636877614685), new Pose(72.35771833528445, 85.18616227619187)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(72.35771833528445, 85.18616227619187), new Pose(55.10318550148585, 51.790292275291364)))
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }
}
