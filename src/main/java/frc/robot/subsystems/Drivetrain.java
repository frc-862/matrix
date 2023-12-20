package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.lib.swervelib.Mk3ModuleConfiguration;
import frc.robot.lib.swervelib.Mk3SwerveModuleHelper;
import frc.robot.lib.swervelib.Mk4ModuleConfiguration;
import frc.robot.lib.swervelib.Mk4SwerveModuleHelper;
import frc.robot.lib.swervelib.SwerveModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.DrivetrainConstants.Offsets;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.DrivetrainConstants.Gains;
import frc.robot.Constants.DrivetrainConstants.HeadingGains;
import frc.robot.lib.SparkMaxPIDGains;
import frc.robot.lib.shuffleboard.LightningShuffleboard;
import frc.robot.lib.shuffleboard.LightningShuffleboardPeriodic;

/**
 * The drivetrain subsystem
 */
public class Drivetrain extends SubsystemBase {

    // Creates our swerve kinematics using the robots track width and wheel base
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0));


    // public Pigeon2 gyro;
    public AHRS gyro = new AHRS(SPI.Port.kMXP);

    // Creating our list of module states and module positions
    private SwerveModuleState[] states = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
    private SwerveModulePosition[] modulePositions = {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};

    // Creating new pose, odometry, cahssis speeds
    private Pose2d pose = new Pose2d();
    private Pose2d rawPose = new Pose2d();
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    // Creating our modules
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    // Module steer offsets
    private double FRONT_LEFT_STEER_OFFSET = Offsets.Howitzer.FRONT_LEFT_STEER_OFFSET;
    private double BACK_LEFT_STEER_OFFSET = Offsets.Howitzer.BACK_LEFT_STEER_OFFSET;
    private double FRONT_RIGHT_STEER_OFFSET = Offsets.Howitzer.FRONT_RIGHT_STEER_OFFSET;
    private double BACK_RIGHT_STEER_OFFSET = Offsets.Howitzer.BACK_RIGHT_STEER_OFFSET;

    // Swerve pose esitmator for odometry
    SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getYaw2d(), modulePositions);

    // Creates our drivetrain shuffleboard tab for displaying module data and a periodic shuffleboard for data that doesn't need constant updates
    private ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    private LightningShuffleboardPeriodic periodicShuffleboard;
    private final Mk3ModuleConfiguration swerveConfiguration = new Mk3ModuleConfiguration();

    // PIDController for heading compenstation
    private final PIDController headingController = new PIDController(HeadingGains.kP, HeadingGains.kI, HeadingGains.kD);

    private boolean flipFL = false;
    private boolean flipFR = false;
    private boolean flipBR = false;
    private boolean flipBL = false;
    // Heading compenstaion variables
    // private boolean updatedHeading = false;
    // private double lastGoodheading = 0d;

    // Chassis speeds for the robot
    private ChassisSpeeds outputChassisSpeeds = new ChassisSpeeds();

    private boolean hasLimitChanged = false;


    private boolean initialSync = false;
    private double initialTimeStamp = 0;

    /**
     * Creates a new Drivetrain.
     * 
     */
    public Drivetrain() {
        // gyro = new AHRS(SPI.Port.kMXP);
        // Zero our gyro
        zeroHeading();

        // if (Constants.isHowitzer()) {
        //     FRONT_LEFT_STEER_OFFSET = Offsets.Howitzer.FRONT_LEFT_STEER_OFFSET;
        //     FRONT_RIGHT_STEER_OFFSET = Offsets.Howitzer.FRONT_RIGHT_STEER_OFFSET;
        //     BACK_LEFT_STEER_OFFSET = Offsets.Howitzer.BACK_LEFT_STEER_OFFSET;
        //     BACK_RIGHT_STEER_OFFSET = Offsets.Howitzer.BACK_RIGHT_STEER_OFFSET;
        // }

        // Set our neo module configurations using drive current, steer current, and
        // voltage
        swerveConfiguration.setDriveCurrentLimit(DrivetrainConstants.DRIVE_CURRENT_LIMIT);
        swerveConfiguration.setSteerCurrentLimit(DrivetrainConstants.STEER_CURRENT_LIMIT);
        swerveConfiguration.setNominalVoltage(DrivetrainConstants.NOMINAL_VOLTAGE);
        swerveConfiguration.setDrivePIDGains(new SparkMaxPIDGains(Gains.kP, Gains.kI, Gains.kD, Gains.kF));

        // Making front left module
        frontLeftModule = Mk3SwerveModuleHelper.createNeo(tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0), swerveConfiguration,
                Mk3SwerveModuleHelper.GearRatio.STANDARD, RobotMap.CAN.FRONT_LEFT_DRIVE_MOTOR, RobotMap.CAN.FRONT_LEFT_AZIMUTH_MOTOR, RobotMap.CAN.FRONT_LEFT_CANCODER, FRONT_LEFT_STEER_OFFSET);

        // Making front right module
        frontRightModule = Mk3SwerveModuleHelper.createNeo(tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0), swerveConfiguration,
                Mk3SwerveModuleHelper.GearRatio.STANDARD, RobotMap.CAN.FRONT_RIGHT_DRIVE_MOTOR, RobotMap.CAN.FRONT_RIGHT_AZIMUTH_MOTOR, RobotMap.CAN.FRONT_RIGHT_CANCODER, FRONT_RIGHT_STEER_OFFSET);

        // Making backleft module
        backLeftModule = Mk3SwerveModuleHelper.createNeo(tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0), swerveConfiguration,
                Mk3SwerveModuleHelper.GearRatio.STANDARD, RobotMap.CAN.BACK_LEFT_DRIVE_MOTOR, RobotMap.CAN.BACK_LEFT_AZIMUTH_MOTOR, RobotMap.CAN.BACK_LEFT_CANCODER, BACK_LEFT_STEER_OFFSET);

        // Making back right module
        backRightModule = Mk3SwerveModuleHelper.createNeo(tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0), swerveConfiguration,
                Mk3SwerveModuleHelper.GearRatio.STANDARD, RobotMap.CAN.BACK_RIGHT_DRIVE_MOTOR, RobotMap.CAN.BACK_RIGHT_AZIMUTH_MOTOR, RobotMap.CAN.BACK_RIGHT_CANCODER, BACK_RIGHT_STEER_OFFSET);

        initialTimeStamp = Timer.getFPGATimestamp();

        // Initialize the shuffleboard values and start logging data
        initializeShuffleboard();


        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public void periodic() {
        if (Timer.getFPGATimestamp() - initialTimeStamp < 1) {
            if (initialSync) {
                // Setting start position and creating estimator
                setInitialPose(new Pose2d(0, 0, new Rotation2d()));

                // Setting states of the modules
                states = new SwerveModuleState[] {
                        new SwerveModuleState(frontLeftModule.getDriveVelocity(), frontLeftModule.getPosition().angle),
                        new SwerveModuleState(frontRightModule.getDriveVelocity(), frontRightModule.getPosition().angle),
                        new SwerveModuleState(backLeftModule.getDriveVelocity(), backLeftModule.getPosition().angle),
                        new SwerveModuleState(backRightModule.getDriveVelocity(), backRightModule.getPosition().angle)};

                updateOdometry();
                updateDriveStates(states);
                resetNeoAngle();

                initialSync = true;
            } else {
                states = new SwerveModuleState[] {
                        new SwerveModuleState(frontLeftModule.getDriveVelocity(), frontLeftModule.getPosition().angle),
                        new SwerveModuleState(frontRightModule.getDriveVelocity(), frontRightModule.getPosition().angle),
                        new SwerveModuleState(backLeftModule.getDriveVelocity(), backLeftModule.getPosition().angle),
                        new SwerveModuleState(backRightModule.getDriveVelocity(), backRightModule.getPosition().angle)};
                updateOdometry();
            }
        } else {
            updateOdometry();

            periodicShuffleboard.loop();
            // periodicShuffleboardAuto.loop();

            if (DriverStation.isTeleop() && !hasLimitChanged) {
                int newLimit = 53;
                frontLeftModule.setDriveCurrentLimit(newLimit);
                frontRightModule.setDriveCurrentLimit(newLimit);
                backLeftModule.setDriveCurrentLimit(newLimit);
                backRightModule.setDriveCurrentLimit(newLimit);

                hasLimitChanged = true;
            }

        }
    }

    public double getDriveVelocity() {
        return (frontLeftModule.getDriveVelocity() + frontRightModule.getDriveVelocity() + backLeftModule.getDriveVelocity() + backRightModule.getDriveVelocity()) / 4;
    }

    /**
     * Method to return the heading controller
     * 
     * @return the heading PIDController
     */
    public PIDController getHeadingController() {
        return headingController;
    }


    /**
     * This takes chassis speeds and converts them to module states and then sets states.
     * 
     * @param chassisSpeeds the chassis speeds to convert to module states
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        outputChassisSpeeds = chassisSpeeds;

        // // If were not commanding any thing to the motors, make sure our states speeds are 0
        // if (states != null && chassisSpeeds.vxMetersPerSecond == 0 && chassisSpeeds.vyMetersPerSecond == 0 && chassisSpeeds.omegaRadiansPerSecond == 0) {
        //     states[0].speedMetersPerSecond = 0;
        //     states[1].speedMetersPerSecond = 0;
        //     states[2].speedMetersPerSecond = 0;
        //     states[3].speedMetersPerSecond = 0;

        // } else {
            // If not set the states to the commanded chassis speeds
            states = kinematics.toSwerveModuleStates(outputChassisSpeeds);
        // }

        // if (flipFL) {
        //     states[0].speedMetersPerSecond *= -1;
        // }
        // if (flipFR) {
        //     states[1].speedMetersPerSecond *= -1;
        // }
        // if (flipBL) {
        //     states[2].speedMetersPerSecond *= -1;
        // }
        // if (flipBR) {
        //     states[3].speedMetersPerSecond *= -1;
        // }

        // Sets the states to the modules
        setStates(states);
    }

    public ChassisSpeeds getOutputChassisSpeeds() {
        return outputChassisSpeeds;
    }

    public void flipFL() {
        flipFL = !flipFL;
    }

    public void flipFR() {
        flipFR = !flipFR;
        
    }

    public void flipBL() {
        flipBL = !flipBL;
    }

    public void flipBR() {
        flipBR = !flipBR;
    }

    public Rotation2d getDriveHeading(double xMeters, double yMeters) {
        double changeX = pose.getX() - xMeters;
        double changeY = pose.getY() - yMeters;

        return Rotation2d.fromRadians(Math.atan2(changeY, changeX));
    }

    /**
     * This takes a list of module states and sets them to the modules.
     * 
     * @param states the list of module states to set
     */
    public void updateDriveStates(SwerveModuleState[] states) {
        if (states != null) {
            SwerveModuleState frontLeftState = states[0];
            SwerveModuleState frontRightState = states[1];
            SwerveModuleState backLeftState = states[2];
            SwerveModuleState backRightState = states[3];

            // Normalize the wheel speeds if the magnitude of any wheel is greater than max velocity
            SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);

            LightningShuffleboard.setDouble("drive", "fl speed", frontLeftState.speedMetersPerSecond);

            // Sets the states to the modules
            frontLeftModule.set(frontLeftState.speedMetersPerSecond, frontLeftState.angle.getRadians());
            frontRightModule.set(frontRightState.speedMetersPerSecond, frontRightState.angle.getRadians());
            backLeftModule.set(backLeftState.speedMetersPerSecond, backLeftState.angle.getRadians());
            backRightModule.set(backRightState.speedMetersPerSecond, backRightState.angle.getRadians());
        }
    }

    /**
     * Updates odometry using the current yaw and module states.
     */
    public void updateOdometry() {
        updateModulePositions();
        pose = odometry.update(getYaw2d(), modulePositions);

        // if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
        //     pose = new Pose2d(16.48 - pose.getX(), pose.getY(), pose.getRotation());
        // }
    }
    /**
     * 2 Method to set states of modules.
     */
    public void setStates(SwerveModuleState[] newStates) {
        states = newStates;
        updateOdometry();
        updateDriveStates(states);

    }

    /**
     * Updates the module positions array to the current positions of each module
     */
    public void updateModulePositions() {
        modulePositions[0] = frontLeftModule.getPosition();
        modulePositions[1] = frontRightModule.getPosition();
        modulePositions[2] = backLeftModule.getPosition();
        modulePositions[3] = backRightModule.getPosition();
    }

    // Method to start sending values to the dashboard and start logging
    @SuppressWarnings("unchecked")
    private void initializeShuffleboard() {
        periodicShuffleboard = new LightningShuffleboardPeriodic("Drivetrain", DrivetrainConstants.LOG_PERIOD, 
            new Pair<String, Object>("Gyro Yaw", (DoubleSupplier) () -> getYaw2d().getDegrees()),
            new Pair<String, Object>("fl module position", (DoubleSupplier) () -> modulePositions[0].distanceMeters),
            new Pair<String, Object>("fr module position", (DoubleSupplier) () -> modulePositions[1].distanceMeters),
            new Pair<String, Object>("bl module position", (DoubleSupplier) () -> modulePositions[2].distanceMeters),
            new Pair<String, Object>("br module position", (DoubleSupplier) () -> modulePositions[3].distanceMeters),
            new Pair<String, Object>("fl amperage", (DoubleSupplier) () -> frontLeftModule.getDriveAmperage()),
            new Pair<String, Object>("fr amperage", (DoubleSupplier) () -> frontRightModule.getDriveAmperage()),
            new Pair<String, Object>("bl amperage", (DoubleSupplier) () -> backLeftModule.getDriveAmperage()),
            new Pair<String, Object>("br amperage", (DoubleSupplier) () -> backRightModule.getDriveAmperage()),
            new Pair<String, Object>("odo Pose", (Supplier<double[]>) () -> new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()}),
            new Pair<String, Object>("raw Pose", (Supplier<double[]>) () -> new double[] {rawPose.getX(), rawPose.getY(), rawPose.getRotation().getRadians()}));
    }



    /**
     * Sets initial pose of robot in meters.
     * 
     * @param initalPosition the initial position of the robot
     */
    public void setInitialPose(Pose2d initalPosition) {
        // gyro.setAngleAdjustment(initalPosition.getRotation().minus(gyro.getRotation2d());
        pose = new Pose2d(initalPosition.getTranslation(), initalPosition.getRotation());
    }

    /**
     * Gets the heading of the robot from odometry in rotation 2d
     */
    public Rotation2d getHeading() {
        return pose.getRotation();
    }

    /**
     * Gets the current rotation from the pigeon
     * 
     * @return the current heading of the robot in degrees from 0 to 360
     */
    public Rotation2d getYaw2d() {
        return Rotation2d.fromDegrees(MathUtil.inputModulus(gyro.getYaw() - 90, 0, 360));
    }

    /**
     * Gets the current pitch of the robot from the pigeon
     * 
     * @return the current pitch of the robot in degrees from -180 to 180
     */
    public Rotation2d getPitch2d() {
        return Rotation2d.fromDegrees(MathUtil.inputModulus(gyro.getPitch(), -180, 180));
    }

    /**
     * Gets the current roll of the robot from the pigeon
     * 
     * @return the current roll of the robot in degrees from -180 to 180
     */
    public Rotation2d getRoll2d() {
        return Rotation2d.fromDegrees(MathUtil.inputModulus(gyro.getRoll(), -180, 180));
    }

    /**
     * Gets current state of module.
     * 
     * @return the current state of the specified module
     */
    public SwerveModuleState stateFromModule(SwerveModule swerveModule) {
        return new SwerveModuleState(swerveModule.getDriveVelocity(), new Rotation2d(swerveModule.getSteerAngle()));
    }

    /**
     * Converts percent output of joystick to a velocity in meters per second.
     * 
     * @param percentOutput the percent output of the joystick
     * 
     * @return the velocity in meters per second
     */
    public double percentOutputToMetersPerSecond(double percentOutput) {
        return percentOutput * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;
    }

    /**
     * Converts percent output of joystick to a rotational velocity in omega radians per second.
     * 
     * @param percentOutput the percent output of the joystick
     * 
     * @return
     */
    public double percentOutputToRadiansPerSecond(double percentOutput) {
        return percentOutput * DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    }

    /**
     * Zeroes the yaw of the pigeon.
     */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Gets the current pose of the robot.
     */
    public Pose2d getPose() {
        return pose;
    }

    /**
     * Resets the odometry to the specified pose.
     * 
     * @param pose the pose to which to set the odometry
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getYaw2d(), modulePositions, pose);
    }

    public void poseReset(Pose2d pose) {
        odometry.resetPosition(getHeading(), modulePositions, pose);
    }

    /**
     * Gets the kinematics of the robot.
     * 
     * @return the kinematics of the robot
     */
    public SwerveDriveKinematics getDriveKinematics() {
        return kinematics;
    }

    /**
     * Gets the states of the modules.
     * 
     * @return the states of the modules
     */
    public SwerveModuleState[] getStates() {
        return states;
    }

    /**
     * Gets the front left module.
     * 
     * @return the front left module
     */
    public SwerveModule getFrontLeftModule() {
        return frontLeftModule;
    }

    /**
     * Gets the front right module.
     * 
     * @return the front right module
     */
    public SwerveModule getFrontRightModule() {
        return frontRightModule;
    }

    /**
     * Gets the back left module.
     * 
     * @return the back left module
     */
    public SwerveModule getBackLeftModule() {
        return backLeftModule;
    }

    /**
     * Gets the back right module.
     * 
     * @return the back right module
     */
    public SwerveModule getBackRightModule() {
        return backRightModule;
    }

    /**
     * Gets the chassis speeds.
     * 
     * @return the chassis speeds
     */
    public ChassisSpeeds getChassisSpeeds() {
        return chassisSpeeds;
    }

    /**
     * Sets the chassis speeds.
     * 
     * @param chassisSpeeds the chassis speeds to set
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    /**
     * Gets if it is in community
     * 
     * @return if in community or not
     */
    public boolean isInCommunity() {
        if (DriverStation.getAlliance() == Alliance.Blue) {
            if ((1.35 < pose.getX() && pose.getX() < 3.35) && (1.50 < pose.getY() && pose.getY() < 5.25) || // Box 1
                    (1.35 < pose.getX() && pose.getX() < 4.85) && (0.00 < pose.getY() && pose.getY() < 1.50) || // Box 2
                    (9.85 < pose.getX() && pose.getX() < 13.20) && (6.78 < pose.getY() && pose.getY() < 7.99) || // Box 3
                    (13.20 < pose.getX() && pose.getX() < 16.25) && (5.51 < pose.getY() && pose.getY() < 7.99)) { // Box 4
                return true;
            }
        } else if (DriverStation.getAlliance() == Alliance.Red) {
            if ((1.35 < pose.getX() && pose.getX() < 3.35) && (2.40 < pose.getY() && pose.getY() < 6.45) || // Box 1
                    (1.35 < pose.getX() && pose.getX() < 4.85) && (6.45 < pose.getY() && pose.getY() < 8.02) || // Box 2
                    (9.85 < pose.getX() && pose.getX() < 13.20) && (0.03 < pose.getY() && pose.getY() < 1.24) || // Box 3
                    (13.20 < pose.getX() && pose.getX() < 16.25) && (0.03 < pose.getY() && pose.getY() < 2.51)) { // Box 4
                return true;
            }
        }
        return false;
    }

    public boolean isInLoadZone() {
        if (DriverStation.getAlliance() == Alliance.Blue) {
            if ((9.85 < pose.getX() && pose.getX() < 13.20) && (6.78 < pose.getY() && pose.getY() < 7.99) || // Box 3
                    (13.20 < pose.getX() && pose.getX() < 16.25) && (5.51 < pose.getY() && pose.getY() < 7.99)) { // Box 4
                return true;
            }
        } else if (DriverStation.getAlliance() == Alliance.Red) {
            if ((9.85 < pose.getX() && pose.getX() < 13.20) && (0.03 < pose.getY() && pose.getY() < 1.24) || // Box 3
                    (13.20 < pose.getX() && pose.getX() < 16.25) && (0.03 < pose.getY() && pose.getY() < 2.51)) { // Box 4
                return true;
            }
        }
        return false;
    }

    /**
     * Sets all motor speeds to 0 and sets the modules to their respective resting angles
     */
    public void stop() {
        frontLeftModule.set(0, DrivetrainConstants.FRONT_LEFT_RESTING_ANGLE);
        frontRightModule.set(0, DrivetrainConstants.FRONT_RIGHT_RESTING_ANGLE);
        backLeftModule.set(0, DrivetrainConstants.BACK_LEFT_RESTING_ANGLE);
        backRightModule.set(0, DrivetrainConstants.BACK_RIGHT_RESTING_ANGLE);

    }

    public void resetNeoAngle() {
        frontLeftModule.setEncoderAngle();
        frontRightModule.setEncoderAngle();
        backLeftModule.setEncoderAngle();
        backRightModule.setEncoderAngle();
    }
}
