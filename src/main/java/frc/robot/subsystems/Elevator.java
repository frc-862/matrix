package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.config.NeoConfig;
import frc.thunder.config.SparkMaxPIDGains;

public class Elevator extends SubsystemBase {
    private CANSparkMax motor;
    private SparkMaxPIDController elevatorController;
    private RelativeEncoder encoder;
    private double targetHeight;

    public Elevator() {
        motor = NeoConfig.createMotor(CAN.ELEVATOR_MOTOR, ElevatorConstants.MOTOR_INVERT,
                ElevatorConstants.CURRENT_LIMIT, Constants.VOLTAGE_COMP_VOLTAGE,
                ElevatorConstants.MOTOR_TYPE, ElevatorConstants.NEUTRAL_MODE);
        elevatorController = NeoConfig.createPIDController(motor.getPIDController(),
                new SparkMaxPIDGains(ElevatorConstants.kP, ElevatorConstants.kI,
                        ElevatorConstants.kD, ElevatorConstants.kF));
        encoder = NeoConfig.createBuiltinEncoder(motor, ElevatorConstants.ENCODER_INVERT);
        encoder.setPositionConversionFactor(ElevatorConstants.POSITION_CONVERSION_FACTOR);

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    /**
     * getExtension
     * 
     * @return the extension distance of the elevator in inches
     */
    public double getExtension() {
        return encoder.getPosition();
    }

    /**
     * setDistance
     * 
     * @param target the target distance in inches
     */
    public void setDistance(double target) {
        // TODO: looks at this, since the elevator is a relative encoder we might not be able to re-zero at the top if its outside of the range
        targetHeight = MathUtil.clamp(target, ElevatorConstants.MIN_HEIGHT,
                ElevatorConstants.MAX_HEIGHT);
        elevatorController.setReference(
                (targetHeight),
                CANSparkMax.ControlType.kPosition);
    }

    /**
     * setPower
     * 
     * @param speed the percent speed to set the elevator motor to
     */
    public void setPower(double speed) {
        motor.set(speed);
    }

    /**
     * stop set the elevator motor to 0% output
     */
    public void stop() {
        setPower(0d);
    }

    /**
     * onTarget
     * 
     * @return true if the elevator is within the tolerance of the target
     */
    public boolean onTarget() {
        return Math.abs(targetHeight - encoder.getPosition()) < ElevatorConstants.TOLERANCE;
    }

    /**
     * getBottomLimitSwitch
     * 
     * @return true if the bottom limit switch is pressed
     */
    public boolean getBottomLimitSwitch() {
        return motor.getReverseLimitSwitch(ElevatorConstants.BOTTOM_LIMIT_SWITCH_TYPE).isPressed();
    }

    /**
     * getTopLimitSwitch
     * 
     * @return true if the top limit switch is pressed
     */
    public boolean getTopLimitSwitch() {
        return motor.getForwardLimitSwitch(ElevatorConstants.TOP_LIMIT_SWITCH_TYPE).isPressed();
    }

    /**
     * isReachable
     * 
     * @param targetHeight the target height in inches
     * 
     * @return true if the target height is reachable by the elevator
     */
    public boolean isReachable(double targetHeight) {
        return targetHeight >= ElevatorConstants.MIN_HEIGHT
                && targetHeight <= ElevatorConstants.MAX_HEIGHT;
    }

    @Override
    public void periodic() {
        if (getTopLimitSwitch()) {
            encoder.setPosition(ElevatorConstants.MAX_HEIGHT);
        }

        if (getBottomLimitSwitch()) {
            encoder.setPosition(ElevatorConstants.MIN_HEIGHT);
        }

    }
}
