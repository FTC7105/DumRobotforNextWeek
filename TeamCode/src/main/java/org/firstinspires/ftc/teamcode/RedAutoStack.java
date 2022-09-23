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

    DcMotor leftfront;
    DcMotor leftback;
    DcMotor rightfront;
    DcMotor rightback;
    Servo fourbar;
    Servo claw;



    @Override
    public void runOpMode() throws InterruptedException {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        fourbar = hardwareMap.get(Servo.class, "fourbar");
        claw = hardwareMap.get(Servo.class, "claw");

        claw.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(180));


        Trajectory DepoPreload = drive.trajectoryBuilder(startPose, 180)
                .splineTo(
                        new Vector2d(-36, -40), Math.toRadians(130),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();


        drive.followTrajectory(DepoPreload);






    }
}
