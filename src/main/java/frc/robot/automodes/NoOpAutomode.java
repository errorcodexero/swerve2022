package frc.robot.automodes;

// caution: it's named "NoOp" because it isn't operating anything. in past years it was named "Nop"...
public class NoOpAutomode extends SwerveDriveAutoMode {
    public NoOpAutomode(SwerveDriveRobotAutoController ctrl) {
        super(ctrl, "nop-auto-mode") ;
    }
}
