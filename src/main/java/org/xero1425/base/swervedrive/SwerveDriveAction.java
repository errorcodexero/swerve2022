package org.xero1425.base.swervedrive;

import org.xero1425.base.actions.Action;

public abstract class SwerveDriveAction extends Action {
        /// \brief Create the object holding a reference to the subsystem
    /// \param drive the tankdrive subsystem
    public SwerveDriveAction(SwerveDriveSubsystem drive) {
        super(drive.getRobot().getMessageLogger()) ;
        swerverive_ = drive ;
    }

    /// \brief return the tank drive subsystem
    /// returns the tank drive subsystem
    public SwerveDriveSubsystem getSubsystem() {
        return swerverive_ ;
    }

    private SwerveDriveSubsystem swerverive_ ;
}
