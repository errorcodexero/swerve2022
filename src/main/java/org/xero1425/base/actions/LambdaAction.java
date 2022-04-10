package org.xero1425.base.actions;

import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

public class LambdaAction extends Action {
    @FunctionalInterface
    public interface ActionFunction 
    {
        void evaluate() ;
    }

    public interface DoneFunction
    {
        boolean evaluate() ;
    }

    private ActionFunction func_ ;
    private DoneFunction done_ ;
    private String name_ ;

    public LambdaAction(MessageLogger logger, String name, ActionFunction dowork) {
        super(logger) ;

        name_ = name ;
        func_ = dowork ;
        done_ = null ;
    }

    public LambdaAction(MessageLogger logger, String name, ActionFunction dowork, DoneFunction done) {
        super(logger) ;

        name_ = name ;
        func_ = dowork ;
        done_ = done ;
    }

    @Override
    public void start() {
        getMessageLogger().startMessage(MessageType.Debug).add("Starting Lambda Action").endMessage();
        func_.evaluate() ;

        if (done_ == null || done_.evaluate() == true)
            setDone() ;

        getMessageLogger().startMessage(MessageType.Debug).add("Ending Start Lambda Action").endMessage();
    }

    @Override
    public void run() {
        getMessageLogger().startMessage(MessageType.Debug).add("Running Lambda Action").endMessage();
        if (done_.evaluate())
            setDone() ;

        getMessageLogger().startMessage(MessageType.Debug).add("ENding RUnning Lambda Action").endMessage();            
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + name_ ;
    }
}
