package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;


public class RunIntakeIn extends Command{
    
    public Intake m_intake;

    public RunIntakeIn(Intake subsystem){
        m_intake = subsystem;
        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        m_intake.runIntake(150);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
    }

    public boolean isFinished(){
        return false;
    }
}
