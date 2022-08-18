package frc.robot.subsystems.gpm;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.intake2motor.Intake2MotorSubsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorSubsystem;

import frc.robot.subsystems.shooter.ShooterSubsystem;

public class GPMSubsystem extends Subsystem {

    private Intake2MotorSubsystem intake_;
    private MotorSubsystem agitator_;
    private ShooterSubsystem shooter_;

    public GPMSubsystem(Subsystem parent, String name) throws Exception {
        super(parent, name);

        intake_ = new Intake2MotorSubsystem(this, "intake");
        addChild(intake_);
        agitator_ = new MotorSubsystem(this, "agitator");
        addChild(agitator_);
        shooter_ = new ShooterSubsystem(this);
        addChild(shooter_);
        //TODO Auto-generated constructor stub
    }

    public Intake2MotorSubsystem getIntake() {
        return intake_;
    }
    public MotorSubsystem getAgitator() {
        return agitator_;
    }
    
    
}
