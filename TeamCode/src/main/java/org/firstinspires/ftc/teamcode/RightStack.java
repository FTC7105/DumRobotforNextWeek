package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RedAuto.FEET_PER_METER;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTag.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class RightStack extends LinearOpMode {

    Servo fourbar;
    Servo claw;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    double fx = 1278.272;
    double fy = 1278.272;
    double cx = 1102.145;
    double cy = 401.506;

    double clawOpen = .65;
    double clawClose = .8;

    double fourbarpickup = .965;
    double fourbarlowpost = .9;

    double tagsize = 0.166;

    int ID_TAG_OF_INTERESTA = 285;
    int ID_TAG_OF_INTERESTB = 286;
    int ID_TAG_OF_INTERESTC = 287;

    boolean tagFoundA = false;
    boolean tagFoundB = false;
    boolean tagFoundC = false;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        fourbar = hardwareMap.get(Servo.class, "fourbar");
        claw = hardwareMap.get(Servo.class, "claw");

        claw.setDirection(Servo.Direction.REVERSE);

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == ID_TAG_OF_INTERESTA) {
                        tagOfInterest = tag;
                        tagFoundA = true;
                        break;
                    } else if (tag.id == ID_TAG_OF_INTERESTB) {
                        tagOfInterest = tag;
                        tagFoundB = true;
                        break;
                    }  else if (tag.id == ID_TAG_OF_INTERESTC) {
                        tagOfInterest = tag;
                        tagFoundC = true;
                        break;
                    }
                }

                if (tagFoundA || tagFoundB || tagFoundC) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }


        Pose2d startPose = new Pose2d(36, -65, Math.toRadians(180));

        Trajectory DepoPreload2 = drive.trajectoryBuilder(startPose)
                .strafeRight(40)
                .build();
        Trajectory PrepareForCycle = drive.trajectoryBuilder(new Pose2d(36, -24, Math.toRadians(180)))
                .strafeRight(12)
                .build();

        Trajectory Backup = drive.trajectoryBuilder(new Pose2d(36, -24, Math.toRadians(180)))
                .back(4)
                .build();

        Trajectory forward = drive.trajectoryBuilder(new Pose2d(32, -24, Math.toRadians(180)))
                .forward(4)
                .build();

        Trajectory PickUp = drive.trajectoryBuilder(new Pose2d(36, -12, Math.toRadians(180)))
                .forward(19.5)
                .build();

        Trajectory dropoff1 = drive.trajectoryBuilder(new Pose2d(55.5, -12, Math.toRadians(180)))
                .back(19)
                .build();

        Trajectory droppoff2 = drive.trajectoryBuilder(new Pose2d(36, -12, Math.toRadians(180)))
                .strafeLeft(12)
                .build();

        Trajectory parkforA = drive.trajectoryBuilder(new Pose2d(36, -12, Math.toRadians(180)))
                .forward(23)
                .build();

        Trajectory parkforC = drive.trajectoryBuilder(new Pose2d(36, -12, Math.toRadians(180)))
                .back(25)
                .build();

        waitForStart();

        if (tagFoundA == true) {
            //  tagOfInterest = tag;
            telemetry.addLine("Pos A");
            drive.setPoseEstimate(startPose);
            telemetry.update();
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
            fourbar.setPosition(1);
            drive.followTrajectory(parkforA);
        } else if (tagFoundB == true) {
            //   tagOfInterest = tag;
            telemetry.addLine("Pos B");
            drive.setPoseEstimate(startPose);
            telemetry.update();
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
        } else if (tagFoundC == true) {
            //    tagOfInterest = tag;
            telemetry.addLine("Pos C");
            drive.setPoseEstimate(startPose);
            telemetry.update();
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
            fourbar.setPosition(1);
            drive.followTrajectory(parkforC);
        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
