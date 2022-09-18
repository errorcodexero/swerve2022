package frc.robot.subsystems.shooter;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;

public class ShooterWheelSubsystem extends MotorEncoderSubsystem {
    public ShooterWheelSubsystem(Subsystem parent, String name, boolean angle, int samples) throws Exception {
        super(parent, name, angle, samples) ;
        setVelocityConversion(1.0);
    } 

    @Override
    public double getVelocity() {
        return super.getVelocity() * 0.366210938 ;
    }
}
