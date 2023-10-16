package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Config
@Autonomous(group = "Autonomous")


public class NearStackUnderTrussAuto extends LinearOpMode {


    enum State{
        IDLE,

        INITIAL_MOVE,

//        MOVE_ACROSS,
//
//        PIXEL_PLACING,
//
//        CHECKING

    }

    double startPoseX = 0;

    double startPoseY = 0;

    double startPoseAngle = 0;

    Pose2d startPose = new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseAngle));

    Pose2d posEstimate;

    //Add variables here:
//        double initialPoseX = 0;
//        double initialPoseY = 0;
//        double initialPoseAngle = 0;

    double spikePoseX = 29.4986;
    double spikePoseY = 0;
    double spikePoseAngle = 0;

    double stackPoseX = 35.0692;
    double stackPoseY = 16.5953;
    double stackPoseAngle = 270;

    double trussBeforePoseX = 0.9615;
    double trussBeforePoseY = -2.2097;
    double trussBeforePoseAngle = 270;

    double trussAfterPoseX = 0.9615;
    double trussAfterPoseY = -43.3513;
    double trussAfterPoseAngle = 270;

    double backdropPoseX = 28.4539;
    double backdropPoseY = -80.2442;
    double backdropPoseAngle = 270;

    double parkPoseX = 0.7625;
    double parkPoseY = -80.8080;
    double parkPoseAngle = 270;

    NearStackUnderTrussAuto.State currentState = NearStackUnderTrussAuto.State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        telemetry.addLine("Init Done");

        //Add Trajectories here:
        TrajectorySequence initialMove = drive.trajectorySequenceBuilder(startPose)
                // The robot is at its starting point.
                .lineToLinearHeading(new Pose2d(spikePoseX, spikePoseY, spikePoseAngle))
                .lineToLinearHeading(new Pose2d(stackPoseX, stackPoseY, stackPoseAngle))
                .lineToLinearHeading(new Pose2d(trussBeforePoseX, trussBeforePoseY, trussBeforePoseAngle))
                .lineToLinearHeading(new Pose2d(trussAfterPoseX, trussAfterPoseY, trussAfterPoseAngle))
                .lineToLinearHeading(new Pose2d(backdropPoseX, backdropPoseY, backdropPoseAngle))
                .lineToLinearHeading(new Pose2d(parkPoseX, parkPoseY, parkPoseAngle))
                .build();

        telemetry.addLine("trajectories built");

        waitForStart();

        while(!isStopRequested() && opModeIsActive()) {
            posEstimate = drive.getPoseEstimate();

            switch(currentState){
                case IDLE:
                    currentState = State.IDLE;

                case INITIAL_MOVE:
                    drive.followTrajectorySequence(initialMove);
                    currentState = State.IDLE;
                    break;
            }
        }
    }
}
