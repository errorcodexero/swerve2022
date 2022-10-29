package frc.robot.subsystems.climber;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;

public class ClimberSubsystem extends MotorEncoderSubsystem {

    public ClimberSubsystem(Subsystem parent) throws Exception {
        super(parent, "climber", false);
    }

    @Override
    protected double limitPower(double p) {
        if (getPosition() > 4600 && p > 0.0) {
            p = 0.0 ;
        }

        return p ;
    }
}