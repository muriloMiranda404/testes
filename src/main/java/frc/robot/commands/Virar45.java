package frc.robot.commands;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;

public class Virar45 extends Command {

    RobotContainer container = new RobotContainer();
    SwerveSubsystem subsystem;
    Translation2d translation2d = new Translation2d(0, 0);
    Pigeon2 pigeon = new Pigeon2(9);
    PIDController controller;
    double MAX_SPEED = 6.0;

    double setpoint;

    public Virar45(SwerveSubsystem subsystem, double setpoint, RobotContainer container){
        if(this.setpoint >= 5000 || this.subsystem == null){
           subsystem.drive(translation2d, 0, true);
        }
        this.subsystem = subsystem;
        this.setpoint = setpoint; 
        this.container = container;
        controller = new PIDController(0.01, 0.0, 0.0);
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        controller.setTolerance(2.0); 
        controller.enableContinuousInput(-180, 180);
    }

    @Override
    public void execute(){
        if(container.controleXbox.getYButton()){
            this.cancel();
        }
        try{
        double yawAtual = pigeon.getYaw().getValueAsDouble();
        double velocidadeRotacao = controller.calculate(yawAtual, setpoint);
        
        // Limita a velocidade de rotação
        velocidadeRotacao = Math.min(Math.max(velocidadeRotacao, -MAX_SPEED), MAX_SPEED);
        
        subsystem.drive(translation2d, velocidadeRotacao, true);
        }
        catch (Exception e){
            System.out.println("algo deu de errado");
            subsystem.drive(translation2d, 0, true);
        }
    }
    @Override
    public void end(boolean interrupted){
        subsystem.drive(translation2d, 0, true);
    }
    @Override
    public boolean isFinished(){
        return controller.atSetpoint();
    }
}