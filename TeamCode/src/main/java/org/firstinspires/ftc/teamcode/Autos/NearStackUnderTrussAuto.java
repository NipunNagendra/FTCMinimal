package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Config
@Autonomous(name = "7.Auto", group = "drive")
public class NearStackUnderTrussAuto {


    enum State{
        IDLE,

        INITIAL_MOVE,

        MOVE_ACROSS,

        PIXEL_PLACING,

        CHECKING

    }

    public class Red40AUTO extends LinearOpMode {
        double startPoseX = 1.7613;

        double startPoseY = 50.7197;

        double startPoseAngle = 359.0065;

        Pose2d startPose = new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseAngle));

        Pose2d posEstimate;

        //Add variables here:
//        double initialPoseX = 0;
//        double initialPoseY = 0;
//        double initialPoseAngle = 0;

        double spikePoseX = 22.6054;
        double spikePoseY = -0.5398;
        double spikePoseAngle = 6.2655;

        double stackPoseX = 29.7505;
        double stackPoseY = 20.4377;
        double stackPoseAngle = 4.6844;

        double trussBeforePoseX = 7.3399;
        double trussBeforePoseY = -3.4015;
        double trussBeforePoseAngle = 4.7291;

        double trussAfterPoseX = 6.7405;
        double trussAfterPoseY = -48.3733;
        double trussAfterPoseAngle = 1.5401;

        double backdropPoseX = 30.6434;
        double backdropPoseY = -86.9907;
        double backdropPoseAngle = 4.7692;

        double parkPoseX = 88.3054;
        double parkPoseY = 1.6608;
        double parkPoseAngle = 6.2434;

        org.firstinspires.ftc.teamcode.Autos.NearStackUnderTrussAuto.State currentState = org.firstinspires.ftc.teamcode.Autos.NearStackUnderTrussAuto.State.IDLE;

        @Override
        public void runOpMode() throws InterruptedException{
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            telemetry.addLine("Init Done");

            //Add Trajectories here:
            TrajectorySequence initialMove = drive.trajectorySequenceBuilder(startPose)
                    // The robot is at its starting point.
                    .lineToLinearHeading(new Pose2d(startPoseX, startPoseY, startPoseAngle))
                    .lineToLinearHeading(new Pose2d(spikePoseX, spikePoseY, spikePoseAngle))
                    .lineToLinearHeading(new Pose2d(stackPoseX, stackPoseY, stackPoseAngle))
                    .lineToLinearHeading(new Pose2d(trussBeforePoseX, trussBeforePoseY, trussBeforePoseAngle))
                    .lineToLinearHeading(new Pose2d(trussAfterPoseX, trussAfterPoseY, trussAfterPoseAngle))
                    .lineToLinearHeading(new Pose2d(backdropPoseX, backdropPoseY, backdropPoseAngle))
                    .lineToLinearHeading(new Pose2d(stackPoseX, stackPoseY, stackPoseAngle))
                    .lineToLinearHeading(new Pose2d(parkPoseX, parkPoseY, parkPoseAngle))
                    .build();

            telemetry.addLine("trajectories built");

            waitForStart();

            while(!isStopRequested() && opModeIsActive()) {
                posEstimate = drive.getPoseEstimate();

                switch(currentState){
                    case IDLE:
                        currentState = org.firstinspires.ftc.teamcode.Autos.NearStackUnderTrussAuto.State.INITIAL_MOVE;

                    case INITIAL_MOVE:
                        drive.followTrajectorySequence(initialMove);
                        currentState = org.firstinspires.ftc.teamcode.Autos.NearStackUnderTrussAuto.State.IDLE;
                        break;
                }
            }
        }
    }
}
