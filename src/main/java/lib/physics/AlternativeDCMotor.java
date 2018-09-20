package lib.physics;

/**
 * Basic DC Motor Model, stolen from PathfinderV2
 * TODO: Invert kV and kT
 */
public class AlternativeDCMotor {

    public final double v_nominal;
    public final double free_speed;
    public final double free_current;
    public final double stall_current;
    public final double stall_torque;

    public AlternativeDCMotor(double v_nominal, double free_speed, double free_current, double stall_current, double stall_torque) {
        this.v_nominal = v_nominal;
        this.free_speed = free_speed;
        this.free_current = free_current;
        this.stall_current = stall_current;
        this.stall_torque = stall_torque;
    }

    public double internal_resistance() {

        // V = I / R, R = I / V, 1 / R = V / I
        // V_per_I = max_V / max_I, 1 / R = V_per_I

        return v_nominal / stall_current;
    }

    public double volt_per_speed() {

        // V_per_I = max_V / max_I
        // free_V = V_per_I * free_I
        // effective_max_V = max_V - free_V
        // V_per_w = effective_max_V / max_w

        return (v_nominal - free_current * v_nominal / stall_current) / free_speed;
    }

    public double current_per_torque() {
        return stall_current / stall_torque;
    }

    public double get_current(double voltage, double speed) {
        // V_vel = kV * w
        double vel_voltage = volt_per_speed() * speed;

        // V = IR + (kV * w) = IR + V_vel
        // I = (V - V_vel) / R

        return (voltage - vel_voltage) / internal_resistance();
    }

    public double get_torque(double current) {
        return current / current_per_torque();
    }

    public double get_free_speed(double voltage) {
        return voltage / volt_per_speed();
    }

    public double get_free_voltage(double speed) {
        return speed * volt_per_speed();
    }

    public double get_current_voltage(double current) {
        return current * internal_resistance();
    }

    public double get_torque_current(double torque) {
        return torque * current_per_torque();
    }

}
