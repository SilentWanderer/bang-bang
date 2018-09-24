package simulation;

import control.DriveOutput;
import control.DrivePlanner;
import lib.geometry.Pose2d;
import lib.geometry.Pose2dWithCurvature;
import lib.physics.WheelState;
import lib.trajectory.TimedView;
import lib.trajectory.Trajectory;
import lib.trajectory.TrajectoryIterator;
import lib.trajectory.timing.TimedState;
import lib.util.ReflectingCSVWriter;
import lib.util.Units;
import odometry.RobotStateEstimator;

public class DriveSimulation {
    
    ReflectingCSVWriter<Pose2d> mOdometryWriter;
    ReflectingCSVWriter<DrivePlanner> mTrajectoryWriter;
    
    RobotStateEstimator mRobotStateEstimator;
    DrivePlanner mDrivePlanner;

    public DriveSimulation(ReflectingCSVWriter<Pose2d> pOdometryWriter, ReflectingCSVWriter<DrivePlanner> pTrajectoryWriter, RobotStateEstimator pRobotStateEstimator, DrivePlanner pDrivePlanner) {
        mOdometryWriter = pOdometryWriter;
        mTrajectoryWriter = pTrajectoryWriter;
        mRobotStateEstimator = pRobotStateEstimator;
        mDrivePlanner = pDrivePlanner;
    }
    
    public double driveTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> pTrajectoryToDrive, double pDt, boolean pResetPoseToTrajectoryStart) {

        if(pResetPoseToTrajectoryStart) {
            mRobotStateEstimator.reset(0.0, pTrajectoryToDrive.getFirstState().state().getPose());
        }

        TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectoryIterator = new TrajectoryIterator<>(new TimedView<>(pTrajectoryToDrive));
        mDrivePlanner.setTrajectory(trajectoryIterator);

        double time = 0.0;
        WheelState wheelDisplacement = new WheelState();

        for (time = 0.0; !mDrivePlanner.isDone(); time += pDt) {

            Pose2d currentPose = mRobotStateEstimator.getRobotState().getLatestFieldToVehiclePose();
            DriveOutput output = mDrivePlanner.update(time, currentPose);

            mTrajectoryWriter.add(mDrivePlanner);
            mOdometryWriter.add(currentPose);

            mTrajectoryWriter.flush();
            mOdometryWriter.flush();


            System.out.println(currentPose);
            System.out.println(output);

            // Our pose estimator expects input in inches, not radians. We happily oblige.
            output = output.rads_to_inches(Units.meters_to_inches(mDrivePlanner.getRobotProfile().getWheelRadiusMeters()));

            // Update the total distance each wheel has traveled, in inches.
            wheelDisplacement = new WheelState(wheelDisplacement.left + (output.left_velocity * pDt) + (0.5 * output.left_accel * pDt * pDt),
                    wheelDisplacement.right + (output.right_velocity * pDt) + (0.5 * output.right_accel * pDt * pDt));

            mRobotStateEstimator.update(time, wheelDisplacement.left, wheelDisplacement.right);
        }

        System.out.println("Total time: " + time);

        return time;
    }

}
