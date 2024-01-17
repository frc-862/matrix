package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.AbsoluteDrive;
import frc.robot.lib.LightningContainer;
import frc.robot.lib.shuffleboard.LightningShuffleboard;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer extends LightningContainer {
    private Drivetrain drivetrain;

    private static XboxController driverController = new XboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
    private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

    @Override
	protected void initializeSubsystems() {
		LightningShuffleboard.set("Auton", "Auto Chooser", autoChooser);
    }

    @Override
    protected void configureButtonBindings() {
        
        this.drivetrain = new Drivetrain();
        drivetrain.zeroGyro();
        new Trigger(driverController::getStartButton).onTrue(new InstantCommand(drivetrain::zeroGyro));
        new Trigger(driverController::getXButton).onTrue(new InstantCommand(drivetrain::lock, drivetrain));
    }

    @Override
    protected Command getAutonomousCommands() {
        return autoChooser.getSelected();
    }

    @Override
    protected void configureDefaultCommands() {
        drivetrain.setDefaultCommand(
            new AbsoluteDrive(
                drivetrain,
                () -> MathUtil.applyDeadband(-driverController.getLeftX(), ControllerConstants.DEADBAND),
                () -> MathUtil.applyDeadband(driverController.getLeftY(), ControllerConstants.DEADBAND),
                () -> -driverController.getRightX(),
                () -> -driverController.getRightY()));
    }

    @Override
    protected void configureSystemTests() {
    }

    @Override
    protected void releaseDefaultCommands() {}

    @Override
    protected void initializeDashboardCommands() {}

    @Override
    protected void configureFaultCodes() {}

    @Override
    protected void configureFaultMonitors() {}
}
