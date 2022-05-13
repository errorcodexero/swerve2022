package frc.robot.oi ;

import org.xero1425.base.Subsystem;
import org.xero1425.base.oi.OISubsystem;
import org.xero1425.base.swervedrive.SwerveDriveSubsystem;

public class Swerve2021OISubsystem extends OISubsystem {
    public final static String SubsystemName = "swerveoi";

    public Swerve2021OISubsystem(Subsystem parent, SwerveDriveSubsystem db) {
        super(parent, SubsystemName, GamePadType.Swerve, db) ;
    }
}
