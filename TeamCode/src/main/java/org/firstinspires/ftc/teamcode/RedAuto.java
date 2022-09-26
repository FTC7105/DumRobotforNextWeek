package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTag.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class RedAuto extends LinearOpMode {


    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    double fx = 1278.272;  // top two increase the z axis if you increase the value
    double fy = 1278.272;
    double cx = 1102.145;
    double cy = 401.506;


//    double fx = 578.272; measurements for small camera
//    double fy = 578.272;
//    double cx = 402.145;
//    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTERESTA = 285; // these are the numbers for the qr code, all of them should be right but it will only sense these three qr codes.
    int ID_TAG_OF_INTERESTB = 286;
    int ID_TAG_OF_INTERESTC = 287;

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

            /*
             * The INIT-loop:
             * This REPLACES waitForStart!
             */
            while (!isStarted() && !isStopRequested()) {
                ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

                if (currentDetections.size() != 0) {
                    boolean tagFound = false;

                    for (AprilTagDetection tag : currentDetections) {
                        if (tag.id == ID_TAG_OF_INTERESTA) {
                            tagOfInterest = tag;
                            tagFound = true;    // this if/else if statements are for the three different sensing levels also the tagFound boolean is not actually necessary
                            telemetry.addData("Position 1", ID_TAG_OF_INTERESTA);
                            break;
                        } else if (tag.id == ID_TAG_OF_INTERESTB) {
                            tagOfInterest = tag;
                            tagFound = true;
                            telemetry.addData("Position 2", ID_TAG_OF_INTERESTB);
                            break;
                        } else if (tag.id == ID_TAG_OF_INTERESTC) {
                            tagOfInterest = tag;
                            tagFound = true;
                            telemetry.addData("Position 3", ID_TAG_OF_INTERESTC);
                            break;
                        }
                    }
                }
                telemetry.update();
            }
    }
}
