package control;

import lib.geometry.Pose2d;
import lib.geometry.Pose2dWithCurvature;
import lib.geometry.Rotation2d;
import lib.geometry.State;
import lib.physics.ChassisState;
import lib.physics.DCMotorTransmission;
import lib.physics.DifferentialDrive;
import lib.trajectory.*;
import lib.trajectory.timing.DifferentialDriveDynamicsConstraint;
import lib.trajectory.timing.TimedState;
import lib.trajectory.timing.TimingConstraint;
import lib.trajectory.timing.TimingUtil;
import lib.util.CSVWritable;
import lib.util.Units;
import lib.util.Util;
import profiles.RobotProfile;

import java.sql.Time;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

public class DrivePlanner implements CSVWritable {

    // Maximum cross-track error - helps prevent unattainable velocity/acceleration commands
    private static final double kMaxDx = 2.0;
    private static final double kMaxDy = 0.25;
    private static final double kMaxDTheta = Math.toRadians(5.0);
    
    public final RobotProfile mRobotProfile;
    public final DCMotorTransmission mDriveTransmission;
    public final DifferentialDrive mDriveModel;
    
    public DrivePlanner(RobotProfile pRobotProfile) {
        mRobotProfile = pRobotProfile;

        // Invert our feedforward constants. Torque constant is kT = I * kA, where I is the robot modeled as a cylindrical load on the motor and kA is the inverted feedforward.
        mDriveTransmission = new DCMotorTransmission(1 / mRobotProfile.getVoltPerSpeed(), mRobotProfile.getCylindricalMoi() / mRobotProfile.getVoltPerAccel(), mRobotProfile.getFrictionVoltage());
        mDriveModel = new DifferentialDrive(mRobotProfile.getLinearInertia(), mRobotProfile.getAngularInertia(), mRobotProfile.getAngularDrag(), mRobotProfile.getWheelRadiusMeters(), mRobotProfile.getWheelbaseRadiusMeters(),
                mDriveTransmission, mDriveTransmission);
    }
    
    // Trajectory and errors are in inches
    TrajectoryIterator<TimedState<Pose2dWithCurvature>> mCurrentTrajectory;
    public TimedState<Pose2dWithCurvature> mSetpoint = new TimedState<>(Pose2dWithCurvature.identity());
    Pose2d mError = Pose2d.identity();
    boolean mIsReversed = false;

    // Rad / s. Taken from previous dynamics output (for now)
    ChassisState prev_velocity_ = new ChassisState();
    double mLastTime = Double.POSITIVE_INFINITY;
    double mDt = 0.0;

    DriveOutput mOutput = new DriveOutput();

    public void setTrajectory(final TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        mCurrentTrajectory = trajectory;
        mSetpoint = trajectory.getState();
        for (int i = 0; i < trajectory.trajectory().length(); ++i) {
            if (trajectory.trajectory().getState(i).velocity() > Util.kEpsilon) {
                mIsReversed = false;
                break;
            } else if (trajectory.trajectory().getState(i).velocity() < -Util.kEpsilon) {
                mIsReversed = true;
                break;
            }
        }
    }

    public void reset() {
        mError = Pose2d.identity();
        mOutput = new DriveOutput();
        mLastTime = Double.POSITIVE_INFINITY;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return generateTrajectory(reversed, waypoints, constraints, 0.0, 0.0, max_vel, max_accel, max_voltage);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel,
            double end_vel,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        List<Pose2d> waypoints_maybe_flipped = waypoints;
        final Pose2d flip = Pose2d.fromRotation(new Rotation2d(-1, 0, false));
        // TODO re-architect the spline generator to support reverse.
        if (reversed) {
            waypoints_maybe_flipped = new ArrayList<>(waypoints.size());
            for (int i = 0; i < waypoints.size(); ++i) {
                waypoints_maybe_flipped.add(waypoints.get(i).transformBy(flip));
            }
        }

        // Create a trajectory from splines.
        Trajectory<Pose2dWithCurvature> trajectory = TrajectoryUtil.trajectoryFromSplineWaypoints(
                waypoints_maybe_flipped, kMaxDx, kMaxDy, kMaxDTheta);

        if (reversed) {
            List<Pose2dWithCurvature> flipped = new ArrayList<>(trajectory.length());
            for (int i = 0; i < trajectory.length(); ++i) {
                flipped.add(new Pose2dWithCurvature(trajectory.getState(i).getPose().transformBy(flip), -trajectory
                        .getState(i).getCurvature(), trajectory.getState(i).getDCurvatureDs()));
            }
            trajectory = new Trajectory<>(flipped);
        }
        // Create the constraint that the robot must be able to traverse the trajectory without ever applying more
        // than the specified voltage.
        final DifferentialDriveDynamicsConstraint<Pose2dWithCurvature> drive_constraints = new
                DifferentialDriveDynamicsConstraint<>(mDriveModel, max_voltage);
        List<TimingConstraint<Pose2dWithCurvature>> all_constraints = new ArrayList<>();
        all_constraints.add(drive_constraints);
        if (constraints != null) {
            all_constraints.addAll(constraints);
        }
        // Generate the timed trajectory.
        Trajectory<TimedState<Pose2dWithCurvature>> timed_trajectory = TimingUtil.timeParameterizeTrajectory
                (reversed, new
                        DistanceView<>(trajectory), kMaxDx, all_constraints, start_vel, end_vel, max_vel, max_accel);
        return timed_trajectory;
    }

    @Override
    public String toCSV() {
        DecimalFormat fmt = new DecimalFormat("#0.000");
        return fmt.format(mOutput.left_velocity) + "," + fmt.format(mOutput.right_velocity) + "," + fmt.format
                (mOutput.left_feedforward_voltage) + "," + fmt.format(mOutput.right_feedforward_voltage) + "," +
                mSetpoint.toCSV();
    }

    public DriveOutput update(double timestamp, Pose2d current_state) {
        if (mCurrentTrajectory == null) return new DriveOutput();

        if (mCurrentTrajectory.getProgress() == 0.0 && !Double.isFinite(mLastTime)) {
            mLastTime = timestamp;
        }

        mDt = timestamp - mLastTime;
        mLastTime = timestamp;
        TrajectorySamplePoint<TimedState<Pose2dWithCurvature>> sample_point = mCurrentTrajectory.advance(mDt);
        mSetpoint = sample_point.state();

        if (!mCurrentTrajectory.isDone()) {
            // Generate feedforward voltages.
            final double velocity_m = Units.inches_to_meters(mSetpoint.velocity());
            final double curvature_m = Units.meters_to_inches(mSetpoint.state().getCurvature());
            final double dcurvature_ds_m = Units.meters_to_inches(Units.meters_to_inches(mSetpoint.state()
                    .getDCurvatureDs()));
            final double acceleration_m = Units.inches_to_meters(mSetpoint.acceleration());
            final DifferentialDrive.DriveDynamics dynamics = mDriveModel.solveInverseDynamics(
                    new ChassisState(velocity_m, velocity_m * curvature_m),
                    new ChassisState(acceleration_m,
                            acceleration_m * curvature_m + velocity_m * velocity_m * dcurvature_ds_m));
            mError = current_state.inverse().transformBy(mSetpoint.state().getPose());

            mOutput = new DriveOutput(dynamics.wheel_velocity.left, dynamics.wheel_velocity.right, dynamics
                    .wheel_acceleration.left, dynamics.wheel_acceleration.right, dynamics.voltage
                    .left, dynamics.voltage.right);

        } else {
            // TODO Possibly switch to a pose stabilizing controller?
            mOutput = new DriveOutput();
        }
        return mOutput;
    }

    public boolean isDone() {
        return mCurrentTrajectory != null && mCurrentTrajectory.isDone();
    }

    public Pose2d error() {
        return mError;
    }

    public TimedState<Pose2dWithCurvature> setpoint() {
        return mSetpoint;
    }
    
    
}
