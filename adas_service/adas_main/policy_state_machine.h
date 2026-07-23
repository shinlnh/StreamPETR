#ifndef POLICY_STATE_MACHINE_H
#define POLICY_STATE_MACHINE_H


#include "adas_main.h"

/**
 * The State machine will have 2 parts: State , and Task:
 * 
 * - State: a context that indicate the object can and can not do if certain criteria met. 
 * State machine has many states.
 * Each time state machine switch its state, the machine's behavior change.
 * 
 * - Task: an implementation of what state machine should do in a state. A single state can not handle all tasks.
 * Some tasks is belonged only to some specific states.
 * 
 * If the state encounters a task that it cannot handle, 
 * The state will hand over that task to another state that can execute it. (function transitionToState)
 * 
 * Only the state-machine (AdasMain) has the implement how to transitionToState.
 * 
 */

/**
 * @brief The PolicyState class represents a pure virtual state in the AdasMain state machine. (a bare-bone class for inherited)
 *
 * This Policy State serves as a pure base class for various states, providing common data members and
 * pure virtual functions that derived classes must implement.
 */
class AdasMain::PolicyState
{
    protected:
        boost::mutex state_machine_mut;
        static LogUtility log; /**< Single static logger shared between all states. */
	    AdasMain *adas_main_ = nullptr; /**< Pointer to the AdasMain instance for ref variables and functions. */
        std::vector<std::shared_ptr<PerceptionOutputObject>> perception_output_objects_queue_;
        std::vector<std::shared_ptr<CameraImageData>> camera_image_data_queue_;
        bool danger_check = false; // This variable is used to cross-check the vehicle status among other statuses

    public:
        virtual ~PolicyState() = default; /** @brief Destructor virtual function, now do nothing, can be overide to adapt with state's need */
        void setMachineRef(AdasMain *adas_main_ptr);
        void execute();
        static void enableLogFile() { log.enableLogFile(); }
        
        /**
         * @brief The enter function to allow different states to do some setup after transitioning.
         * @param adas_main_ptr 
         */
        virtual void enter(AdasMain *adas_main_ptr) = 0;
		
		/** @brief Base Idle task handle function */
        virtual void doIdle();
        
        /** @brief Base LKS task handle function */
		virtual void doLKS();
		
		/** @brief Base TJA task handle function */
        virtual void doTJA();
		
		/** @brief Base ACC task handle function */
        virtual void doACC();

        /** @brief Base TJC task handle function */
        virtual void doTJC();

        /** @brief Base HWC task handle function */
        virtual void doHWC();
		
		/** @brief Base Brake task handle function */
        virtual void doBrake();

        virtual int getId() = 0;
        virtual bool laneKeepEnabled() { return false; }
        virtual bool speedKeepEnabled() { return false; }
        
        virtual const char* getStateName() = 0;
};


/* ========= ALL STATE CLASSS ========*/

/**
 * Each state will have its implementation to each pure virtual function of PolicyState.
 * The rule is that :
 * - Single state is only handle some functions that belong to that state's scope.
 * - If a tasks isn't belong to that state's scope, transition to another state that can handle that task.
 */

/**
 * @brief The IdleState class represents the idle state. Its main task is doIdle().
 * It inherits from the base PolicyState class and provides implementations for the pure virtual functions.
 */
class AdasMain::IdleState : public AdasMain::PolicyState {
public:
    void enter(AdasMain *adas_main_ptr) override;
    void doIdle() override;
    int getId() override { return IDLE_STATE_ID; }
    const char* getStateName() override { return "Idle State"; }
};

/**
 * @brief The BrakeState class represents the brake state. Its main task is doBrake().
 * It inherits from the base PolicyState class and provides implementations for the pure virtual functions.
 */
class AdasMain::BrakeState : public AdasMain::PolicyState {
private:
    bool release = false;
    bool checkReleaseAEB();
public:
    void doLKS() override;
    void doTJA() override;
    void doACC() override;
    void doHWC() override;
    void doTJC() override;
    void doIdle() override;
    void doBrake() override;
    void enter(AdasMain *adas_main_ptr) override;
    int getId() override { return BRAKE_STATE_ID; }
    const char* getStateName() override { return "Brake State"; }
    void notifyReleaseAEB();
};

/**
 * @brief The LKSState class represents the Lane Tracking state in the AdasMain state machine.
 * It inherits from the base PolicyState class and provides implementations for the pure virtual functions.
 * Its main task is doLKS()
 */
class AdasMain::LKSState : public AdasMain::PolicyState {
public:
    void enter(AdasMain *adas_main_ptr) override;
    void doLKS() override;
    void doTJA() override;
    void doACC() override;
    int getId() override { return LKS_STATE_ID; }
    bool laneKeepEnabled() { return true; }
    const char* getStateName() override { return "LKS State"; }
};


/**
 * @brief The TJAState class represents the Traffic Jam state in the AdasMain state machine.
 * It inherits from the base PolicyState class and provides implementations for the pure virtual functions.
 * Its main task is doTJA()
 */
class AdasMain::TJAState : public AdasMain::PolicyState {
public:
    void enter(AdasMain *adas_main_ptr) override;
    void doLKS() override;
    void doTJA() override;
    int getId() override { return TJA_STATE_ID; }
    bool speedKeepEnabled() { return true; }
    const char* getStateName() override { return "TJA State"; };
};

/**
 * @brief The ACCState class represents the Manual Drive with AEB state in the AdasMain state machine.
 * It inherits from the base PolicyState class and provides implementations for the pure virtual functions.
 * Its main task is doACC()
 */

class AdasMain::ACCState : public AdasMain::PolicyState {
public:
    void enter(AdasMain *adas_main_ptr) override;
    void doLKS() override;
    void doACC() override;
    int getId() override { return ACC_STATE_ID; }
    bool speedKeepEnabled() { return true; }
    const char* getStateName() override { return "ACC State"; };
};

/**
 * @brief The HWCState class represents Highway Chauffeur in the AdasMain state machine.
 * It inherits from the base PolicyState class and provides implementations for the pure virtual functions.
 * Its main task is doHWC()
 */

class AdasMain::HWCState : public AdasMain::PolicyState {
public:
    void enter(AdasMain *adas_main_ptr) override;
    void doHWC() override;
    int getId() override { return HWC_STATE_ID; }
    bool laneKeepEnabled() { return true; }
    bool speedKeepEnabled() { return true; }
    const char* getStateName() override { return "HWC State"; };
};

/**
 * @brief The TJCState class represents Traffic Jam Chauffeur in the AdasMain state machine.
 * It inherits from the base PolicyState class and provides implementations for the pure virtual functions.
 * Its main task is doTJC()
 */

class AdasMain::TJCState : public AdasMain::PolicyState {
public:
    void enter(AdasMain *adas_main_ptr) override;
    void doTJC() override;
    int getId() override { return TJC_STATE_ID; }
    bool laneKeepEnabled() { return true; }
    bool speedKeepEnabled() { return true; }
    const char* getStateName() override { return "TJC State"; };
};

#endif