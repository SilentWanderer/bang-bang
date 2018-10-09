package control;

import lib.geometry.Pose2d;
import lib.geometry.Pose2dWithCurvature;
import lib.geometry.Rotation2d;
import lib.trajectory.TimedView;
import lib.trajectory.Trajectory;
import lib.trajectory.TrajectoryIterator;
import lib.trajectory.timing.CentripetalAccelerationConstraint;
import lib.trajectory.timing.TimedState;
import lib.trajectory.timing.TimingConstraint;
import lib.util.ReflectingCSVWriter;
import odometry.Kinematics;
import odometry.RobotState;
import odometry.RobotStateEstimator;
import paths.TrajectoryGenerator;
import paths.autos.NearScaleAuto;
import profiles.LockdownProfile;
import profiles.RobotProfile;
import simulation.DriveSimulation;

import java.util.Arrays;
import java.util.List;

/**
 * High level manager for pose tracking, path/trajectory following, and pose stabilization.
 */
public class DriveController {

    private final double kDt;

    private final RobotProfile mRobotProfile;
    private final DriveMotionPlanner mDriveMotionPlanner;
    private final Kinematics mKinematicModel;
    private final RobotStateEstimator mRobotStateEstimator;

    public DriveController(RobotProfile pRobotProfile, double pDt) {
        this.mRobotProfile = pRobotProfile;
        this.kDt = pDt;
        this.mDriveMotionPlanner = new DriveMotionPlanner(mRobotProfile, DriveMotionPlanner.PlannerMode.FEEDFORWARD_ONLY);
        this.mKinematicModel = new Kinematics(mRobotProfile);
        this.mRobotStateEstimator = new RobotStateEstimator(mKinematicModel);
    }

    public DriveOutput getOutput(double pTimestamp, double pLeftAbsolutePos, double pRightAbsolutePos) {
        mRobotStateEstimator.update(pTimestamp, pLeftAbsolutePos, pRightAbsolutePos);

        return mDriveMotionPlanner.update(pTimestamp, mRobotStateEstimator.getRobotState().getLatestFieldToVehiclePose());
    }

    public DriveController setPlannerMode(DriveMotionPlanner.PlannerMode pPlannerMode) {
        mDriveMotionPlanner.setPlannerMode(pPlannerMode);
        return this;
    }

    public DriveController setPose(Pose2d pPose) {
        mRobotStateEstimator.reset(0.0, pPose);
        return this;
    }

    public DriveController setTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> pTrajectory, boolean pResetToTrajectoryStart) {
        TrajectoryIterator<TimedState<Pose2dWithCurvature>> iterator = new TrajectoryIterator<>(new TimedView<>(pTrajectory));
        mDriveMotionPlanner.setTrajectory(iterator);

        if(pResetToTrajectoryStart) {
            mRobotStateEstimator.reset(pTrajectory.getFirstState().t(), pTrajectory.getFirstState().state().getPose());
        }

        return this;
    }

    public DriveController setRotationTrajectory(Trajectory<TimedState<Rotation2d>> pTrajectory) {
        TrajectoryIterator<TimedState<Rotation2d>> iterator = new TrajectoryIterator<>(new TimedView<>(pTrajectory));
        mDriveMotionPlanner.setRotationTrajectory(iterator);

        return this;
    }

    public boolean isDone() {
        return mDriveMotionPlanner.isDone();
    }

    public DriveMotionPlanner getDriveMotionPlanner() {
        return mDriveMotionPlanner;
    }

    public RobotStateEstimator getRobotStateEstimator() {
        return mRobotStateEstimator;
    }

    public RobotProfile getRobotProfile() {
        return mRobotProfile;
    }
}
