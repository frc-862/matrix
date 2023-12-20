package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.lib.shuffleboard.LightningShuffleboard;

public class Elevator extends SubsystemBase {

    TalonFX elevatorMotor1;
    TalonFX elevatorMotor2;

    public Elevator() {
        elevatorMotor1 = new TalonFX(CAN.ELEVATOR_MOTOR_1);
        elevatorMotor2 = new TalonFX(CAN.ELEVATOR_MOTOR_2);

        MotorOutputConfigs config = new MotorOutputConfigs();
        config.NeutralMode = NeutralModeValue.Brake;
        
        elevatorMotor1.getConfigurator().apply(config);
        elevatorMotor2.getConfigurator().apply(config);

    }

    @Override
    public void periodic() {
        LightningShuffleboard.setDouble("Elevator", "Motor 1 RPM", (elevatorMotor1.getVelocity().asSupplier().get() / 60));
        LightningShuffleboard.setDouble("Elevator", "Motor 2 RPM", (elevatorMotor2.getVelocity().asSupplier().get() / 60));

        LightningShuffleboard.setDouble("Elevator", "Motor 1 Current", elevatorMotor1.getTorqueCurrent().asSupplier().get());
        LightningShuffleboard.setDouble("Elevator", "Motor 2 Current", elevatorMotor2.getTorqueCurrent().asSupplier().get());

        LightningShuffleboard.setDouble("Elevator", "Motor 1 Target Power", elevatorMotor1.get());
        LightningShuffleboard.setDouble("Elevator", "Motor 2 Target Power", elevatorMotor2.get());

        LightningShuffleboard.setDouble("Elevator", "Motor 1 Position raw", elevatorMotor1.getPosition().getValue()); 
    }

    public void setPower(double power) {
        elevatorMotor1.set(power);
        elevatorMotor2.set(power);
    }

    public void stop() {
        setPower(0d);
    }

    /**
     * 
     * @return rotations of the Elevator
     */
    public double getPosition() {
        return elevatorMotor1.getPosition().getValue();
    }
}
