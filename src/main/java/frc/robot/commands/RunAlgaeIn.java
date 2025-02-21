package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaeRemover;

public class RunAlgaeIn extends Command{
    
    public algaeRemover m_algae;

    public RunAlgaeIn(algaeRemover subsystem){
        m_algae = subsystem;
        addRequirements(m_algae);
    }
    
    @Override
    public void execute() {
        m_algae.runAlgaeRemover(200);
    }

    @Override
    public void end(boolean interrupted) {
        m_algae.stop();
    }

    public boolean isFinished(){
        return false;
    }
}
