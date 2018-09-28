package control;

import lib.geometry.*;
import lib.physics.ChassisState;
import lib.physics.DifferentialDrive;
import lib.physics.WheelState;
import lib.trajectory.TrajectoryIterator;
import lib.trajectory.timing.TimedState;
import lib.util.Units;
import lib.util.Util;

public class NonlinearFeedbackController extends AController {


    public NonlinearFeedbackController(DifferentialDrive pDriveModel) {
        super(pDriveModel);
    }

    @Override
    public DriveOutput update(TrajectoryIterator<TimedState<Pose2dWithCurvature>> pCurrentTrajectory,
                              TimedState<Pose2dWithCurvature> pSetpoint,
                              DifferentialDrive.DriveDynamics pDynamics,
                              ChassisState pPrevVelocity,
                              Pose2d pCurrentState,
                              double pDt) {

        mError = pCurrentState.inverse().transformBy(pSetpoint.state().getPose());

        // Implements eqn. 5.12 from https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf
        final double kBeta = 2.0;  // >0.
        final double kZeta = 0.7;  // Damping coefficient, [0, 1].

        // Compute gain parameter.
        final double k = 2.0 * kZeta * Math.sqrt(kBeta * pDynamics.chassis_velocity.linear * pDynamics.chassis_velocity
                .linear + pDynamics.chassis_velocity.angular * pDynamics.chassis_velocity.angular);

        // Compute error components.
        final double angle_error_rads = mError.getRotation().getRadians();
        final double sin_x_over_x = Util.epsilonEquals(angle_error_rads, 0.0, 1E-2) ?
                1.0 : mError.getRotation().sin() / angle_error_rads;
        final ChassisState adjusted_velocity = new ChassisState(
                pDynamics.chassis_velocity.linear * mError.getRotation().cos() +
                        k * Units.inches_to_meters(mError.getTranslation().x()),
                pDynamics.chassis_velocity.angular + k * angle_error_rads +
                        pDynamics.chassis_velocity.linear * kBeta * sin_x_over_x * Units.inches_to_meters(mError
                                .getTranslation().y()));

        // Compute adjusted left and right wheel velocities.
        pDynamics.chassis_velocity = adjusted_velocity;
        pDynamics.wheel_velocity = mDriveModel.solveInverseKinematics(adjusted_velocity);

        pDynamics.chassis_acceleration.linear = pDt == 0 ? 0.0 : (pDynamics.chassis_velocity.linear - pPrevVelocity
                .linear) / pDt;
        pDynamics.chassis_acceleration.angular = pDt == 0 ? 0.0 : (pDynamics.chassis_velocity.angular - pPrevVelocity
                .angular) / pDt;

        WheelState feedforward_voltages = mDriveModel.solveInverseDynamics(pDynamics.chassis_velocity,
                pDynamics.chassis_acceleration).voltage;

        return new DriveOutput(pDynamics.wheel_velocity.left, pDynamics.wheel_velocity.right, pDynamics.wheel_acceleration
                .left, pDynamics.wheel_acceleration.right, feedforward_voltages.left, feedforward_voltages.right);
        
        
    }

}
