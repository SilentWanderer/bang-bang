package simulation;

import control.DriveOutput;
import control.DrivePlanner;
import lib.geometry.Pose2d;
import lib.geometry.Pose2dWithCurvature;
import lib.geometry.Rotation2d;
import lib.physics.WheelState;
import lib.trajectory.TimedView;
import lib.trajectory.Trajectory;
import lib.trajectory.TrajectoryIterator;
import lib.trajectory.timing.TimedState;
import lib.util.ReflectingCSVWriter;
import lib.util.Units;
import lib.util.Util;
import odometry.RobotStateEstimator;

public class DriveSimulation {
    
    ReflectingCSVWriter<Pose2d> mOdometryWriter;
    ReflectingCSVWriter<DrivePlanner> mTrajectoryWriter;
    
    RobotStateEstimator mRobotStateEstimator;
    DrivePlanner mDrivePlanner;
    WheelState mWheelDisplacement = new WheelState();
    double time = 0.0;


    public DriveSimulation(ReflectingCSVWriter<Pose2d> pOdometryWriter, ReflectingCSVWriter<DrivePlanner> pTrajectoryWriter, RobotStateEstimator pRobotStateEstimator, DrivePlanner pDrivePlanner) {
        mOdometryWriter = pOdometryWriter;
        mTrajectoryWriter = pTrajectoryWriter;
        mRobotStateEstimator = pRobotStateEstimator;
        mDrivePlanner = pDrivePlanner;
    }

    public double driveTrajectory(Trajectory<TimedState<Rotation2d>> pTrajectoryToDrive, double pDt) {
        TrajectoryIterator<TimedState<Rotation2d>> trajectoryIterator = new TrajectoryIterator<>(new TimedView<>(pTrajectoryToDrive));
        mDrivePlanner.setRotationTrajectory(trajectoryIterator);

        for (; !mDrivePlanner.isDone(); time += pDt) {

            Pose2d currentPose = mRobotStateEstimator.getRobotState().getLatestFieldToVehiclePose();
            DriveOutput output = mDrivePlanner.update(time, currentPose);

            mTrajectoryWriter.add(mDrivePlanner);
            mOdometryWriter.add(currentPose);

            mTrajectoryWriter.flush();
            mOdometryWriter.flush();

            if(Math.abs(output.left_feedforward_voltage) > 12.0 || Math.abs(output.right_feedforward_voltage) > 12.0) {
                System.err.println("Warning: Output above 12.0 volts.");
                output.left_velocity = Util.limit(output.left_velocity, 12.0);
                output.right_velocity = Util.limit(output.right_velocity, 12.0);
            }

            // Our pose estimator expects input in inches, not radians. We happily oblige.
            output = output.rads_to_inches(Units.meters_to_inches(mDrivePlanner.getRobotProfile().getWheelRadiusMeters()));

            // Update the total distance each wheel has traveled, in inches.
            mWheelDisplacement = new WheelState(mWheelDisplacement.left + (output.left_velocity * pDt) + (0.5 * output.left_accel * pDt * pDt),
                    mWheelDisplacement.right + (output.right_velocity * pDt) + (0.5 * output.right_accel * pDt * pDt));

            mRobotStateEstimator.update(time, mWheelDisplacement.left, mWheelDisplacement.right);
        }

        System.out.println("Total time: " + time);

        return time;
    }

    public double driveTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> pTrajectoryToDrive, double pDt, boolean pResetPoseToTrajectoryStart) {

        if(pResetPoseToTrajectoryStart) {
            mRobotStateEstimator.reset(pTrajectoryToDrive.getFirstState().t(), pTrajectoryToDrive.getFirstState().state().getPose());
        }

        TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectoryIterator = new TrajectoryIterator<>(new TimedView<>(pTrajectoryToDrive));
        mDrivePlanner.setTrajectory(trajectoryIterator);

        for (; !mDrivePlanner.isDone(); time += pDt) {

            Pose2d currentPose = mRobotStateEstimator.getRobotState().getLatestFieldToVehiclePose();
            DriveOutput output = mDrivePlanner.update(time, currentPose);

            mTrajectoryWriter.add(mDrivePlanner);
            mOdometryWriter.add(currentPose);

            mTrajectoryWriter.flush();
            mOdometryWriter.flush();

            if(Math.abs(output.left_feedforward_voltage) > 12.0 || Math.abs(output.right_feedforward_voltage) > 12.0) {
                System.err.println("Warning: Output above 12.0 volts.");
                output.left_velocity = Util.limit(output.left_velocity, 12.0);
                output.right_velocity = Util.limit(output.right_velocity, 12.0);
            }

            // Our pose estimator expects input in inches, not radians. We happily oblige.
            output = output.rads_to_inches(Units.meters_to_inches(mDrivePlanner.getRobotProfile().getWheelRadiusMeters()));

            // Update the total distance each wheel has traveled, in inches.
            mWheelDisplacement = new WheelState(mWheelDisplacement.left + (output.left_velocity * pDt) + (0.5 * output.left_accel * pDt * pDt),
                    mWheelDisplacement.right + (output.right_velocity * pDt) + (0.5 * output.right_accel * pDt * pDt));

            mRobotStateEstimator.update(time, mWheelDisplacement.left, mWheelDisplacement.right);
        }

        System.out.println("Total time: " + time);

        return time;
    }

}
