package org.xero1425.base.controllers;

import org.xero1425.base.actions.SequenceAction;

/// \file

/// \brief This class defines an auto mode that can be run by the
/// AutoController.  A derived class is expected to be created that contains the
/// specifics of the automode.
public class AutoMode extends SequenceAction {
    
    // The name of the auto mode
    private String name_ ;

    // The auto mode controller
    private AutoController ctrl_ ;

    /// \brief Create a new automode object
    /// \param ctrl the automode controller that manages this auto mode
    /// \param name the name of the automode
    public AutoMode(AutoController ctrl, String name) {
        super(ctrl.getRobot().getMessageLogger()); 

        ctrl_ = ctrl ;
        name_ = name ;

        ctrl.addAutoMode(this) ;
    }

    /// \brief Returns the name of the automode
    /// \returns the name of the automode
    public String getName() {
        return name_ ;
    }

    /// \brief Update the automode.  This is called once per robot loop while the
    /// robot is disabled and initializes the automode.
    /// \param gamedata the gamedata from the WPILib driver station game dat APIs.
    public void update(String gamedata) throws Exception {
    }

    /// \brief Returns the auto mode controller that manages this automode
    /// \returns the auto mode controller that manages this automode
    protected AutoController getAutoController() {
        return ctrl_ ;
    }

}