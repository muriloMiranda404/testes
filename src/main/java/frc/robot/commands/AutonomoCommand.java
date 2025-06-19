package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class AutonomoCommand extends SequentialCommandGroup {
    public AutonomoCommand() {
        addCommands(
            new InstantCommand(() -> frc.robot.Robot.setpoint_intake = 71.0),
            new WaitUntilCommand(() -> frc.robot.Robot.PID_inatake.atSetpoint()),
            new InstantCommand(() -> frc.robot.Robot.setpoint_elevador = 1480.0),
            new WaitUntilCommand(() -> frc.robot.Robot.PIDElevador.atSetpoint()),
            new InstantCommand(() -> frc.robot.Robot.motor_da_bola.set(0.3)),
            new WaitUntilCommand(() -> frc.robot.Robot.fim_de_curso_Coral.get() == false),
            new InstantCommand(() -> frc.robot.Robot.motor_da_bola.set(0))
        );
    }
}