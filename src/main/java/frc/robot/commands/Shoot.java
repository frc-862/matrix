package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.lib.shuffleboard.LightningShuffleboard;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
    
    Indexer indexer;
    Shooter shooter;
    double startShootTime = 0;
    double indexTime = 0.5;
    boolean isShooting = false;

    public Shoot(Indexer indexer, Shooter shooter) {
        this.indexer = indexer;
        this.shooter = shooter;

        addRequirements(shooter);
    }
    @Override
    public void initialize() {
        indexer.setPower(0);
    }

    @Override
    public void execute() {
        shooter.setRPM(LightningShuffleboard.getDouble("shooter", "target RPM", 0));
        
        // if(shooter.getRPM() >= LightningShuffleboard.getDouble("shooter", "target RPM", 1000)) {
        //     indexer.setPower(Constants.INDEX_POWER);
        //     startShootTime = Timer.getFPGATimestamp();
        //     isShooting = true;
        // }

        // shooter.setPower(LightningShuffleboard.getDouble("shooter", "target pow", 0.7));
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setPower(0);
        indexer.setPower(0);
    }

    @Override
    public boolean isFinished() {
        // return Timer.getFPGATimestamp() - startShootTime >= 0.5 && isShooting;
        return false;
    }
}
