import control.DriveOutput;
import control.DrivePlanner;
import lib.geometry.Pose2d;
import lib.geometry.Pose2dWithCurvature;
import lib.geometry.Rotation2d;
import lib.physics.ChassisState;
import lib.physics.DifferentialDrive;
import lib.physics.WheelState;
import lib.trajectory.TimedView;
import lib.trajectory.Trajectory;
import lib.trajectory.TrajectoryIterator;
import lib.trajectory.timing.TimedState;
import lib.trajectory.timing.TimingConstraint;
import lib.util.Units;
import odometry.Kinematics;
import odometry.RobotState;
import odometry.RobotStateEstimator;
import profiles.LockdownProfile;
import profiles.RobotProfile;

import java.util.Arrays;
import java.util.List;

public class DriveSimulation {

    private final double kDt = 1.0 / 100.0;

    private RobotProfile mRobotProfile = new LockdownProfile();
    private DrivePlanner mDrivePlanner = new DrivePlanner(mRobotProfile);
    private Kinematics mKinematicModel = new Kinematics(mRobotProfile);
    private RobotState mRobotState = new RobotState(mKinematicModel);
    private RobotStateEstimator mRobotStateEstimator = new RobotStateEstimator(mRobotState, mKinematicModel);

    private List<Pose2d> mWaypoints = Arrays.asList(new Pose2d[] {
       new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
       new Pose2d(50.0, 50.0, Rotation2d.fromDegrees(45.0)),
       new Pose2d(100.0, 100.0, Rotation2d.fromDegrees(0.0))
    });

    private List<TimingConstraint<Pose2dWithCurvature>> mTrajectoryConstraints = Arrays.asList();

    // in / s
    private final double kMaxVel = 120.0;
    private final double kMaxAccel = 30.0;
    private final double kMaxVoltage = 12.0;

    public void simulate() {

        mDrivePlanner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(generateTrajectory())));

        double time = 0.0;
        WheelState wheelDisplacement = new WheelState();

        for(time = 0.0; !mDrivePlanner.isDone(); time += kDt) {

            DriveOutput output = mDrivePlanner.update(time, mRobotState.getLatestFieldToVehiclePose());

            System.out.println(mRobotState.getLatestFieldToVehiclePose());

            // Our pose estimator expects input in inches, not radians. We happily oblige.
            output = output.rads_to_inches(Units.meters_to_inches(mRobotProfile.getWheelRadiusMeters()));

            // Update the total distance each wheel has traveled, in inches.
            wheelDisplacement = new WheelState(wheelDisplacement.left + (output.left_velocity * kDt) + (0.5 * output.left_accel * kDt * kDt),
                    wheelDisplacement.right + (output.right_velocity * kDt) + (0.5 * output.right_accel * kDt * kDt));

            mRobotStateEstimator.update(time, wheelDisplacement.left, wheelDisplacement.right);
        }

    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory() {
        return mDrivePlanner.generateTrajectory(false, mWaypoints, mTrajectoryConstraints, kMaxVel, kMaxAccel, kMaxVoltage);
    }

}
