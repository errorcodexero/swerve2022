package frc.robot.automodes;

import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.AutoMode;

public abstract class SwerveDriveAutoMode extends AutoMode {
    SwerveDriveAutoMode(AutoController ctrl, String name) {
        super(ctrl, name) ;
    }
}
