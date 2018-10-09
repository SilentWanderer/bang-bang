package simulation;

import control.DriveMotionPlanner;
import control.DriveOutput;
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
    ReflectingCSVWriter<DriveMotionPlanner> mTrajectoryWriter;
    
    RobotStateEstimator mRobotStateEstimator;
    DriveMotionPlanner mDriveMotionPlanner;
    WheelState mWheelDisplacement = new WheelState();
    private final double kDt;
    double time = 0.0;


    public DriveSimulation(ReflectingCSVWriter<Pose2d> pOdometryWriter, ReflectingCSVWriter<DriveMotionPlanner> pTrajectoryWriter, RobotStateEstimator pRobotStateEstimator, DriveMotionPlanner pDriveMotionPlanner, double pDt) {
        mOdometryWriter = pOdometryWriter;
        mTrajectoryWriter = pTrajectoryWriter;
        mRobotStateEstimator = pRobotStateEstimator;
        mDriveMotionPlanner = pDriveMotionPlanner;
        kDt = pDt;
    }

    public double driveTrajectory(Trajectory<TimedState<Rotation2d>> pTrajectoryToDrive) {
        double startTime = time;

        TrajectoryIterator<TimedState<Rotation2d>> trajectoryIterator = new TrajectoryIterator<>(new TimedView<>(pTrajectoryToDrive));
        mDriveMotionPlanner.setRotationTrajectory(trajectoryIterator);

        simulate();

        System.out.println("Trajectory time: " + (time - startTime));

        return time - startTime;
    }

    public double driveTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> pTrajectoryToDrive, boolean pResetPoseToTrajectoryStart) {
        double startTime = time;

        if(pResetPoseToTrajectoryStart) {
            mRobotStateEstimator.reset(pTrajectoryToDrive.getFirstState().t(), pTrajectoryToDrive.getFirstState().state().getPose());
        }

        TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectoryIterator = new TrajectoryIterator<>(new TimedView<>(pTrajectoryToDrive));
        mDriveMotionPlanner.setTrajectory(trajectoryIterator);

        simulate();

        System.out.println("Trajectory time: " + (time - startTime));

        return time - startTime;
    }

    private void simulate() {
        for (; !mDriveMotionPlanner.isDone(); time += kDt) {

            Pose2d currentPose = mRobotStateEstimator.getRobotState().getLatestFieldToVehiclePose();
            DriveOutput output = mDriveMotionPlanner.update(time, currentPose);

            mTrajectoryWriter.add(mDriveMotionPlanner);
            mOdometryWriter.add(currentPose);

            mTrajectoryWriter.flush();
            mOdometryWriter.flush();

            if(Math.abs(output.left_feedforward_voltage) > 12.0 || Math.abs(output.right_feedforward_voltage) > 12.0) {
                System.err.println("Warning: Output above 12.0 volts.");
                // Limit velocity
                output.left_velocity = Util.limit(output.left_velocity, 120.0);
                output.right_velocity = Util.limit(output.right_velocity, 120.0);
            }

            // Our pose estimator expects input in inches, not radians. We happily oblige.
            output = output.rads_to_inches(Units.meters_to_inches(mDriveMotionPlanner.getRobotProfile().getWheelRadiusMeters()));

            // Update the total distance each wheel has traveled, in inches.
            mWheelDisplacement = new WheelState(mWheelDisplacement.left + (output.left_velocity * kDt) + (0.5 * output.left_accel * kDt * kDt),
                    mWheelDisplacement.right + (output.right_velocity * kDt) + (0.5 * output.right_accel * kDt * kDt));

            mRobotStateEstimator.update(time, mWheelDisplacement.left, mWheelDisplacement.right);
        }
    }

    public void setPose(Pose2d pRobotPose) {
        mRobotStateEstimator.reset(0.0, pRobotPose);
    }

}
