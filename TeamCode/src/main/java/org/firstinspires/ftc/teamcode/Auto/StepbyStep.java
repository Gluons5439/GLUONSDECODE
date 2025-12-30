package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "StepbyStep", group = "Examples")
public class StepbyStep extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Create follower
        Follower follower = Constants.createFollower(hardwareMap);

        // Set starting pose
        follower.setStartingPose(new Pose(8, 56, 0));

        // Initial update
        follower.update();

        waitForStart();
        if (isStopRequested()) return;

        // Create all paths
        Paths paths = new Paths(follower);

        CRServo transferServo1; // NEW CR Servo
        CRServo transferServo2; // NEW CR Servo
        transferServo1 = hardwareMap.get(CRServo.class, "Transfer1");
        transferServo2 = hardwareMap.get(CRServo.class, "Transfer2");
        transferServo1.setDirection(DcMotorSimple.Direction.REVERSE);
        transferServo2.setDirection(DcMotorSimple.Direction.REVERSE);

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
        sleep(5000);
        transferServo2.setPower(1);
        sleep(3000);
        transferServo2.setPower(0);
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);


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
        shooterMotor1.setPower(-1);
        shooterMotor2.setPower(-1);
        sleep(5000);
        transferServo2.setPower(1);
        sleep(3000);
        transferServo2.setPower(0);
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);

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
                    .addPath(new BezierLine(new Pose(8, 56), new Pose(36, 56)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(36, 56), new Pose(36, 98.4386208)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(36, 98.4386208), new Pose(23.8, 40.5)))
                    .setTangentHeadingInterpolation()
                    .build();


            Path4 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(23.8, 40.5), new Pose(60, 64)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(60, 64), new Pose(60, 98.4)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(60, 98.4), new Pose(23.5, 40.5)))
                    .setTangentHeadingInterpolation()
                    .build();

        }
    }
}
