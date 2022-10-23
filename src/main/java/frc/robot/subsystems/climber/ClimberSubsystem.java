package frc.robot.subsystems.climber;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;

public class ClimberSubsystem extends MotorEncoderSubsystem {

    public ClimberSubsystem(Subsystem parent) throws Exception {
        super(parent, "climber", false);
    }
}