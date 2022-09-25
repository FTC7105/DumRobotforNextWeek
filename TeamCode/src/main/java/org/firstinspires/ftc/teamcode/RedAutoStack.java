package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous
public class RedAutoStack extends LinearOpMode {

    Servo fourbar;
    Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        fourbar = hardwareMap.get(Servo.class, "fourbar");
        claw = hardwareMap.get(Servo.class, "claw");

        claw.setDirection(Servo.Direction.REVERSE);

        waitForStart();


        Pose2d startPose = new Pose2d(-36,-64,Math.toRadians(180));

//        Trajectory DepoPreload = drive.trajectoryBuilder(startPose, 180)
//                .splineTo(
//                        new Vector2d(-36, -40), Math.toRadians(180),
//                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
//                )
//                .build();
        Trajectory DepoPreload2 = drive.trajectoryBuilder(startPose)
                .strafeRight(40)
                        .build();
        Trajectory PrepareForCycle = drive.trajectoryBuilder(new Pose2d(-36,-24,Math.toRadians(180)))
                .strafeRight(12)
                .build();



        Trajectory Backup= drive.trajectoryBuilder(new Pose2d(-36,-24,Math.toRadians(180)))
                .back(4)
                .build();

        Trajectory forward= drive.trajectoryBuilder(new Pose2d(-32,-24,Math.toRadians(180)))
                .forward(4)
                .build();

        Trajectory PickUp = drive.trajectoryBuilder(new Pose2d(-36,-12,Math.toRadians(180)))
                .forward(18.5)
                .build();

        Trajectory dropoff1 = drive.trajectoryBuilder(new Pose2d(-54.5,-12,Math.toRadians(180)))
                .back(19)
                .build();

        Trajectory droppoff2 = drive.trajectoryBuilder(new Pose2d(-36,-12,Math.toRadians(180)))
                .strafeLeft(12)
                .build();

//adb connect 192.168.43.1:5555

        double clawOpen = .65;
        double clawClose = .8;

        double fourbarpickup = .965;
        double fourbarlowpost = .9;

        drive.setPoseEstimate(startPose);
        // drives to low post

        claw.setPosition(clawClose);
        sleep(1000);
        // Claw Close and

        fourbar.setPosition(fourbarlowpost);
        sleep(1000);
        // 4bar Low Post

        drive.followTrajectory(DepoPreload2);

        drive.followTrajectory(Backup);
        fourbar.setPosition(.93);
        sleep(1000);


        claw.setPosition(clawOpen);
        sleep(1000);
        // Claw Open
        fourbar.setPosition(fourbarlowpost);
        drive.followTrajectory(forward);

        drive.followTrajectory(PrepareForCycle);

        fourbar.setPosition(fourbarpickup);
        //4bar for pickup pos

        drive.followTrajectory(PickUp);

        claw.setPosition(clawClose);
        sleep(1000);


        fourbar.setPosition(fourbarlowpost);
        drive.followTrajectory(dropoff1);
        drive.followTrajectory(droppoff2);

        drive.followTrajectory(Backup);
        fourbar.setPosition(.93);
        sleep(1000);


        claw.setPosition(clawOpen);
        sleep(1000);
        // Claw Open
        fourbar.setPosition(fourbarlowpost);
        drive.followTrajectory(forward);
        drive.followTrajectory(PrepareForCycle);

    }
}
