package paths;

import control.DrivePlanner;
import lib.geometry.Pose2d;
import lib.geometry.Pose2dWithCurvature;
import lib.geometry.Rotation2d;
import lib.trajectory.DistanceView;
import lib.trajectory.Trajectory;
import lib.trajectory.TrajectoryUtil;
import lib.trajectory.WaypointUtil;
import lib.trajectory.timing.DifferentialDriveDynamicsConstraint;
import lib.trajectory.timing.TimedState;
import lib.trajectory.timing.TimingConstraint;
import lib.trajectory.timing.TimingUtil;
import lib.util.Units;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TrajectoryGenerator {

    // Maximum delta between each trajectory point
    private static final double kMaxDx = 2.0;
    private static final double kMaxDy = 0.25;
    private static final double kMaxDTheta = Math.toRadians(5.0);

    private DrivePlanner mDrivePlanner;

    public TrajectoryGenerator(DrivePlanner pDrivePlanner) {
        mDrivePlanner = pDrivePlanner;
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

        List<Pose2d> waypoints_maybe_flipped = (reversed) ? WaypointUtil.flipWaypoints(waypoints) : waypoints;

        // Create a trajectory from splines.
        Trajectory<Pose2dWithCurvature> trajectory = TrajectoryUtil.trajectoryFromSplineWaypoints(
                waypoints_maybe_flipped, kMaxDx, kMaxDy, kMaxDTheta);

        trajectory = (reversed) ? TrajectoryUtil.flip(trajectory) : trajectory;

        // Create the constraint that the robot must be able to traverse the trajectory without ever applying more
        // than the specified voltage.
        final DifferentialDriveDynamicsConstraint<Pose2dWithCurvature> drive_constraints = new
                DifferentialDriveDynamicsConstraint<>(mDrivePlanner.getDriveModel(), max_voltage);
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

    /**
     * Generates rotation trajectories by calculating wheel distances needed to achieve angle, then converting the generated trajectories into rotation states.
     * @param reversed
     * @param initial_heading
     * @param final_heading
     * @param constraints
     * @param end_vel
     * @param max_vel
     * @param max_accel
     * @param max_voltage
     * @return A rotation trajectory for the robot to follow while turning in place.
     */
    public Trajectory<TimedState<Rotation2d>> generateTurnInPlaceTrajectory(Rotation2d initial_heading,
                                                                            Rotation2d final_heading,
                                                                            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
                                                                            double end_vel,
                                                                            double max_vel,  // inches/s
                                                                            double max_accel,  // inches/s^2
                                                                            double max_voltage) {

        Rotation2d rotation_delta = initial_heading.inverse().rotateBy(final_heading);

        // Find distance necessary to move wheels to achieve change in heading
        double distance = rotation_delta.getRadians() * Units.meters_to_inches(mDrivePlanner.getRobotProfile().getWheelbaseRadiusMeters());
        List<Pose2d> wheelTravel = Arrays.asList(new Pose2d(0.0, 0.0, new Rotation2d()),
                new Pose2d(distance, 0.0, new Rotation2d()));

        // Create the constraint that the robot must be able to traverse the trajectory without ever applying more
        // than the specified voltage.
        final DifferentialDriveDynamicsConstraint<Pose2dWithCurvature> drive_constraints = new
                DifferentialDriveDynamicsConstraint<>(mDrivePlanner.getDriveModel(), max_voltage);
        List<TimingConstraint<Pose2dWithCurvature>> all_constraints = new ArrayList<>();
        all_constraints.add(drive_constraints);
        if (constraints != null) {
            all_constraints.addAll(constraints);
        }

        Trajectory<TimedState<Pose2dWithCurvature>> wheelTrajectory = generateTrajectory(false, wheelTravel, all_constraints, 0.0, end_vel, max_vel, max_accel, max_voltage);

        Trajectory<TimedState<Rotation2d>> timedRotationDeltaTrajectory = TrajectoryUtil.distanceToRotation(wheelTrajectory,
                initial_heading,
                Units.meters_to_inches(mDrivePlanner.getRobotProfile().getWheelbaseRadiusMeters()));

        return timedRotationDeltaTrajectory;
    }

    /**
     * For convenience, allow headings to be entered in degrees.
     * @param pInitialHeadingDegrees
     * @param pFinalHeadingDegrees
     * @param pTrajectoryConstraints
     * @param pEndVel
     * @param pMaxVel
     * @param pMaxAccel
     * @param pMaxVoltage
     * @return
     */
    public Trajectory<TimedState<Rotation2d>> generateTurnInPlaceTrajectory(double pInitialHeadingDegrees, double pFinalHeadingDegrees,
                                                                            List<TimingConstraint<Pose2dWithCurvature>> pTrajectoryConstraints,
                                                                            double pEndVel, double pMaxVel, double pMaxAccel, double pMaxVoltage) {
        return generateTurnInPlaceTrajectory(Rotation2d.fromDegrees(pInitialHeadingDegrees), Rotation2d.fromDegrees(pFinalHeadingDegrees),
                pTrajectoryConstraints, pEndVel, pMaxVel, pMaxAccel, pMaxVoltage);
    }

}
