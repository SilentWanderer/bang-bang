package simulation;

import control.DrivePlanner;
import lib.geometry.Pose2d;
import lib.geometry.Pose2dWithCurvature;
import lib.geometry.Rotation2d;
import lib.geometry.Translation2d;
import lib.trajectory.Trajectory;
import lib.trajectory.timing.CentripetalAccelerationConstraint;
import lib.trajectory.timing.TimedState;
import lib.trajectory.timing.TimingConstraint;
import lib.util.ReflectingCSVWriter;
import odometry.Kinematics;
import odometry.RobotState;
import odometry.RobotStateEstimator;
import paths.TrajectoryGenerator;
import paths.autos.NearScaleAuto;
import paths.autos.StartingPoses;
import profiles.LockdownProfile;
import profiles.RobotProfile;

import java.util.Arrays;
import java.util.List;

public class TrackingSimulation {

    private final double kDt = 1.0 / 100.0;

    private RobotProfile mRobotProfile = new LockdownProfile();
    private DrivePlanner mDrivePlanner = new DrivePlanner(mRobotProfile, DrivePlanner.PlannerMode.FEEDBACK);
    private Kinematics mKinematicModel = new Kinematics(mRobotProfile);
    private RobotState mRobotState = new RobotState(mKinematicModel);
    private RobotStateEstimator mRobotStateEstimator = new RobotStateEstimator(mRobotState, mKinematicModel);


    // in / s
    private final double kMaxLinearVel = 130.0; // 10 ft/s -> 120 in / s
    private final double kMaxLinearAccel = 130.0;
    private final double kMaxCentripetalAccel = /*100.0*/70.0;
    private final double kMaxVoltage = 9.0;

    private final List<TimingConstraint<Pose2dWithCurvature>> kTrajectoryConstraints = Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel));

    private TrajectoryGenerator mTrajectoryGenerator = new TrajectoryGenerator(mDrivePlanner);

    public void simulate() {

        ReflectingCSVWriter<Pose2d> csvPoseWriter = new ReflectingCSVWriter<>("tracking.csv", Pose2d.class);
        ReflectingCSVWriter<DrivePlanner> csvDrivePlanner = new ReflectingCSVWriter<>("trajectory.csv", DrivePlanner.class);
        DriveSimulation driveSimulation = new DriveSimulation(csvPoseWriter, csvDrivePlanner, mRobotStateEstimator, mDrivePlanner, kDt);

        double timeDriven = 0.0;

        timeDriven += driveSimulation.driveTrajectory(generate(NearScaleAuto.kToScalePath), true);
        timeDriven += driveSimulation.driveTrajectory(generate(NearScaleAuto.kAtScale.rotation_, NearScaleAuto.kTurnFromScaleToFirstCube.rotation_));
        timeDriven += driveSimulation.driveTrajectory(generate(NearScaleAuto.kScaleToFirstCubePath), false);
        timeDriven += driveSimulation.driveTrajectory(generate(NearScaleAuto.kScaleToFirstCube.rotation_, NearScaleAuto.kTurnFromFirstCubeToScale.rotation_));
        timeDriven += driveSimulation.driveTrajectory(generate(NearScaleAuto.kFirstCubeToScalePath), false);

        System.out.println("Time Driven:" + timeDriven);

    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generate(List<Pose2d> waypoints) {
        return mTrajectoryGenerator.generateTrajectory(false, waypoints, kTrajectoryConstraints, kMaxLinearVel, kMaxLinearAccel, kMaxVoltage);
    }

    public Trajectory<TimedState<Rotation2d>> generate(double initialHeading, double finalHeading ) {
        return mTrajectoryGenerator.generateTurnInPlaceTrajectory(initialHeading, finalHeading, kTrajectoryConstraints,0.0, kMaxLinearVel, kMaxLinearAccel, kMaxVoltage);
    }

    public Trajectory<TimedState<Rotation2d>> generate(Rotation2d initialHeading, Rotation2d finalHeading ) {
        return mTrajectoryGenerator.generateTurnInPlaceTrajectory(initialHeading, finalHeading, kTrajectoryConstraints,0.0, kMaxLinearVel, kMaxLinearAccel, kMaxVoltage);
    }

}
