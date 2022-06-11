package org.xero1425.base.swervedrive;

import org.xero1425.base.actions.Action;

public abstract class SwerveDriveAction extends Action {
    SwerveDriveSubsystem sub_ ;

    public SwerveDriveAction(SwerveDriveSubsystem sub) {
        super(sub.getRobot().getMessageLogger()) ;

        sub_ = sub ;
    }

    protected SwerveDriveSubsystem getSubsystem() {
        return sub_ ;
    }
}
