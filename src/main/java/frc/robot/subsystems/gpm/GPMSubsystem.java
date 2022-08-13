package frc.robot.subsystems.gpm;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.intake2motor.Intake2MotorSubsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorSubsystem;

public class GPMSubsystem extends Subsystem {

    private Intake2MotorSubsystem intake_;
    private MotorSubsystem agitator_;

    public GPMSubsystem(Subsystem parent, String name) throws Exception {
        super(parent, name);

        intake_ = new Intake2MotorSubsystem(this, "intake");
        agitator_ = new MotorSubsystem(this, "agitator");
        //TODO Auto-generated constructor stub
    }

    public Intake2MotorSubsystem getIntake() {
        return intake_;
    }
    public MotorSubsystem getAgitator() {
        return agitator_;
    }
    
    
}
