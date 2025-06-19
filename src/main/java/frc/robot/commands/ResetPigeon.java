package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.imu.Pigeon2Swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

public class ResetPigeon extends Command{

    Pigeon2 pigeon = new Pigeon2(9);
    SwerveSubsystem subsystem;

    public ResetPigeon(Pigeon2 pigeon, SwerveSubsystem subsystem){
        this.pigeon = pigeon;
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }
    @Override
    public void initialize(){
        System.out.println("inicialização do reset");
    }
    @Override
    public void execute(){
        pigeon.reset();
        subsystem.swerveDrive.zeroGyro();
        pigeon.setYaw(subsystem.swerveDrive.getGyroRotation3d().getAngle());
        System.out.println("pigeon reiniciado");
        System.out.println("gyro"+ subsystem.swerveDrive.getGyro().getRawRotation3d().getAngle());
    }
    @Override
    public boolean isFinished(){
        return pigeon.getYaw().getValueAsDouble() == 0;
    }
    @Override
    public void end(boolean interrupted){
        subsystem.drive(new Translation2d(0, 0), 0, true);
    }
}