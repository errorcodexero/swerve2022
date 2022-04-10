package org.xero1425.misc;

/// \file

/// \brief The type of path following used by the robot 
public enum XeroPathType {
    SwervePathFollowing,        ///< Swerve drive using path following
    SwervePurePursuit,          ///< Swerve drive using pure pursuit
    SwerveRamsete,              ///< Swerve drive using ramsete
    TankPathFollowing,          ///< Tank drive using path following
    TankPurePursuit,            ///< Tank drive using pure pursuit
    TankRamsete,                ///< Tank drive using ramsets
}
