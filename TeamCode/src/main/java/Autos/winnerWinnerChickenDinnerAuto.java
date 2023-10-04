package Autos;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.message.redux.StopOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name="winnerWinnerChickenDinnerAuto",group="Autonomous")
public class winnerWinnerChickenDinnerAuto extends LinearOpMode {
    enum State{
        IDLE,
        INITIAL_MOVE
    }
    double startPoseX = -63.2541;
    double startPoseY = -9.3084;
    double startPoseAngle = 0;

    Pose2d startPose=new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseAngle));

    Pose2d posEstimate;

    //add variables here


    State currentState = State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        telemetry.addLine("init done");
//Add trajectories here
        TrajectorySequence goToSpike = drive.trajectorySequenceBuilder(startPose)
                .forward(5)
                .build();
        TrajectorySequence yellowPixelToBackdrop = drive.trajectorySequenceBuilder(goToSpike.end())
                .lineToLinearHeading(new Pose2d())



        telemetry.addLine("TrajectoryBuilt");

        waitForStart();

        while(!isStopRequested() && opModeIsActive()){
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
