package org.firstinspires.ftc.teamcode.testing;

import org.firstinspires.ftc.teamcode.testing.BluePipeline;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


import java.util.jar.Attributes;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="CVRunnerBlue-Nipun", group="TeleOp")
public class CVRunnerBlue extends LinearOpMode {

    private RedPipeline.Location location;

    OpenCvWebcam camera;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "identifyier","teamcode");
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        BluePipeline detector = new BluePipeline(telemetry);
        camera.setPipeline(detector);
        camera.setMillisecondsPermissionTimeout(5000);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {



            }
        });

        FtcDashboard.getInstance().startCameraStream(camera, 0);

        waitForStart();

        switch(detector.getLocation()) {
            case LEFT:
                break;
            case RIGHT:
                break;
            case NOT_FOUND:
                break;
        }

        camera.stopStreaming();
    }
}
