package odometry;

import lib.geometry.Pose2d;
import lib.geometry.Rotation2d;
import lib.geometry.Translation2d;
import lib.geometry.Twist2d;
import lib.physics.DifferentialDrive;
import lib.physics.WheelState;
import lib.util.InterpolatingDouble;
import lib.util.InterpolatingTreeMap;

import java.util.Map;

public class RobotState {

    private static final int kObservationBufferSize = 100;

    private Kinematics mKinematicModel;

    // FPGATimestamp -> RigidTransform2d or Rotation2d
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;
    private Twist2d vehicle_velocity_predicted_;
    private Twist2d vehicle_velocity_measured_;
    private double distance_driven_;

    public RobotState(Kinematics pKinematicModel) {
        this.mKinematicModel = pKinematicModel;
        reset(0, new Pose2d());
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     */
    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle) {
        field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        vehicle_velocity_predicted_ = Twist2d.identity();
        vehicle_velocity_measured_ = Twist2d.identity();
        distance_driven_ = 0.0;
    }

    public synchronized void resetDistanceDriven() {
        distance_driven_ = 0.0;
    }

    /**
     *
     * @param timestamp
     * @return Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
     * to fill in the gaps.
     */
    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    /**
     *
     * @return The latest observation from the pose buffer
     */
    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }

    /**
     *
     * @return The latest observation from the pose buffer, minus the timestamp.
     */
    public synchronized Pose2d getLatestFieldToVehiclePose() {
        return getLatestFieldToVehicle().getValue().getPose();
    }

    /**
     *
     * @param lookahead_time
     * @return The predicted position of the robot based on our stored predicted_velocity.
     */
    public synchronized Pose2d getPredictedFieldToVehicle(double lookahead_time) {
        // dx = v * t
        return getLatestFieldToVehicle().getValue()
                .transformBy(Pose2d.exp(vehicle_velocity_predicted_.scaled(lookahead_time)));
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
        field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservations(double timestamp, Twist2d measured_velocity,
                                             Twist2d predicted_velocity) {
        addFieldToVehicleObservation(timestamp,
                mKinematicModel.integrateForwardKinematics(getLatestFieldToVehicle().getValue(), measured_velocity));
        vehicle_velocity_measured_ = measured_velocity;
        vehicle_velocity_predicted_ = predicted_velocity;
    }

    public synchronized Twist2d generateOdometryFromSensors(double left_encoder_delta_distance, double
            right_encoder_delta_distance, Rotation2d current_gyro_angle) {
        final Pose2d last_measurement = getLatestFieldToVehicle().getValue();
        final Twist2d delta = mKinematicModel.forwardKinematics(last_measurement.getRotation(),
                left_encoder_delta_distance, right_encoder_delta_distance,
                current_gyro_angle);
        distance_driven_ += delta.dx; //do we care about dy here?
        return delta;
    }

    public synchronized Twist2d generateOdometryFromSensors(double left_encoder_delta_distance, double
            right_encoder_delta_distance) {
        final Twist2d delta = mKinematicModel.forwardKinematics(left_encoder_delta_distance, right_encoder_delta_distance);
        distance_driven_ += delta.dx; //do we care about dy here?
        return delta;
    }

    public synchronized double getDistanceDriven() {
        return distance_driven_;
    }

    public synchronized Twist2d getPredictedVelocity() {
        return vehicle_velocity_predicted_;
    }

    public synchronized Twist2d getMeasuredVelocity() {
        return vehicle_velocity_measured_;
    }

}
