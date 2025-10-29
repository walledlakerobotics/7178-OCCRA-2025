package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;


public class ClawSubsystem extends SubsystemBase {
    private final Solenoid m_clampSolenoid;

    public ClawSubsystem() {
        m_clampSolenoid = new Solenoid(
                PneumaticsConstants.kPCMID, 
                PneumaticsModuleType.CTREPCM, 
                PneumaticsConstants.kClawChannel);
        //set later
    }

    public Command toggle() {
        return runOnce( () -> {
            m_clampSolenoid.toggle();
    });

    }
}