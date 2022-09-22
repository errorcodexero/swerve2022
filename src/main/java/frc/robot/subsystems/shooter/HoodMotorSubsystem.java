package frc.robot.subsystems.shooter;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;

public class HoodMotorSubsystem extends MotorEncoderSubsystem {

    public HoodMotorSubsystem(Subsystem parent) throws Exception {
        super(parent, "shooter-hood", false) ;
    }

    @Override
    public void computeMyState() throws Exception {
        super.computeMyState();
    }

    @Override
    public void run() throws Exception {
        super.run() ;
    }

    @Override
    public double limitPower(double p) {
        // if (getPosition() >= getMaxPos() && p > 0.0) {
        //     p = 0.0 ;
        // } if (getPosition() <= getMinPos() && p < 0.0) {
        //     p = 0.0 ;
        // }
        return p ;
    }
}