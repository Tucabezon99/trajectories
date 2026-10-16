//  created:    2025/03/02
//  filename:   trajectories.cpp
//
//  author:     ateveraz
//
//  version:    $Id: 0.1$
//
//  purpose:    Custom control template
//
//
/*********************************************************************/

#include "trajectories.h"
#include <TargetController.h>
#include <Uav.h>
#include <GridLayout.h>
#include <PushButton.h>
#include <DataPlot1D.h>
#include <DataPlot2D.h>
#include <FrameworkManager.h>
#include <VrpnClient.h>
#include <MetaVrpnObject.h>
#include <TrajectoryGenerator2DCircle.h>
#include <Matrix.h>
#include <cmath>
#include <Tab.h>
#include <Pid.h>
#include <Ahrs.h>
#include <AhrsData.h>
#include <GroupBox.h>
#include <ComboBox.h>
#include <CheckBox.h>
#include <DoubleSpinBox.h>
#include <Vector3DSpinBox.h>
#include <iostream>

using namespace std;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::filter;
using namespace flair::meta;

trajectories::trajectories(TargetController *controller): UavStateMachine(controller), behaviourMode(BehaviourMode_t::Default), vrpnLost(false), controlMode_t(ControlMode_t::Default) {
    Uav* uav=GetUav();

    VrpnClient* vrpnclient=new VrpnClient("vrpn", uav->GetDefaultVrpnAddress(),80,uav->GetDefaultVrpnConnectionType());
    
    if(vrpnclient->ConnectionType()==VrpnClient::Xbee) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName(),(uint8_t)0);
        targetVrpn=new MetaVrpnObject("target",1);
    } else if (vrpnclient->ConnectionType()==VrpnClient::Vrpn) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
        targetVrpn=new MetaVrpnObject("target");
    } else if (vrpnclient->ConnectionType()==VrpnClient::VrpnLite) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
        targetVrpn=new MetaVrpnObject("target");
    }
    
    //set vrpn as failsafe altitude sensor for mamboedu as us in not working well for the moment
    if(uav->GetType()=="mamboedu") {
      SetFailSafeAltitudeSensor(uavVrpn->GetAltitudeSensor());
    }
    
    getFrameworkManager()->AddDeviceToLog(uavVrpn);
    getFrameworkManager()->AddDeviceToLog(targetVrpn);
    vrpnclient->Start();
    
    uav->GetAhrs()->YawPlot()->AddCurve(uavVrpn->State()->Element(2),DataPlot::Green);
																 
    startCircle=new PushButton(GetButtonsLayout()->NewRow(),"start_circle");
    stopCircle=new PushButton(GetButtonsLayout()->LastRowLastCol(),"stop_circle");
    positionHold=new PushButton(GetButtonsLayout()->LastRowLastCol(),"position hold");

    // Groupbox for control selection
    controlModeBox = new GroupBox(GetButtonsLayout()->NewRow(),"Control mode");
    on_customController = new PushButton(controlModeBox->NewRow(), "Activate");
    off_customController = new PushButton(controlModeBox->LastRowLastCol(), "Deactivate");
    control_selection = new ComboBox(controlModeBox->NewRow(), "Control selection");
    control_selection->AddItem("Default");
    control_selection->AddItem("Custom controller");

    // Custom tasks
    GroupBox *task_selection_box = new GroupBox(GetButtonsLayout()->LastRowLastCol(), "Custom task");
    task_selection = new ComboBox(task_selection_box->NewRow(), "Custom task");
    task_selection->AddItem("Hovering at zero");
    task_selection->AddItem("Regulation task");
    task_selection->AddItem("Circle tracking");
    task_selection->AddItem("Straight line");

    desired_position = new Vector3DSpinBox(task_selection_box->NewRow(), "Desired position", -3, 3, 0.1, 3);
    desired_yaw = new DoubleSpinBox(task_selection_box->LastRowLastCol(), "Desired yaw", -M_PI, M_PI, 0.1, 3);

    circle=new TrajectoryGenerator2DCircle(vrpnclient->GetLayout()->NewRow(),"circle");
    uavVrpn->xPlot()->AddCurve(circle->GetMatrix()->Element(0,0),DataPlot::Blue);
    uavVrpn->yPlot()->AddCurve(circle->GetMatrix()->Element(0,1),DataPlot::Blue);
    uavVrpn->VxPlot()->AddCurve(circle->GetMatrix()->Element(1,0),DataPlot::Blue);
    uavVrpn->VyPlot()->AddCurve(circle->GetMatrix()->Element(1,1),DataPlot::Blue);
    uavVrpn->XyPlot()->AddCurve(circle->GetMatrix()->Element(0,1),circle->GetMatrix()->Element(0,0),DataPlot::Blue,"circle");

    uX=new Pid(setupLawTab->At(1,0),"u_x");
    uX->UseDefaultPlot(graphLawTab->NewRow());
    uY=new Pid(setupLawTab->At(1,1),"u_y");
    uY->UseDefaultPlot(graphLawTab->LastRowLastCol());

    // Custom control law
    Tab *setup_custom_controller = new Tab(getFrameworkManager()->GetTabWidget(),"Custom controller");
    nuevo = new MyController(setup_custom_controller->At(0,0),"Parameters");

    customReferenceOrientation= new AhrsData(this,"reference");
    uav->GetAhrs()->AddPlot(customReferenceOrientation,DataPlot::Yellow);
    AddDataToControlLawLog(customReferenceOrientation);
    AddDeviceToControlLawLog(uX);
    AddDeviceToControlLawLog(uY);
    AddDeviceToControlLawLog(nuevo);

    customOrientation=new AhrsData(this,"orientation");
}

trajectories::~trajectories() {
}

void trajectories::ComputeCustomTorques(Euler &torques)
{
    // Implement your custom control law here or call a controller class. 
    ComputeDefaultTorques(torques);
    thrust = ComputeDefaultThrust();

    // Selection of the control mode based on the control_selection combobox. 
    switch (control_selection->CurrentIndex())
    {
        case 1:
            controlMode_t = ControlMode_t::Custom;
            computenuevo(torques);
            ComputeCustomThrust();
            break;

        default:
            controlMode_t = ControlMode_t::Default;
            Thread::Warn("trajectories: default control law started. Check custom torque definition. \n");
            EnterFailSafeMode();
            break;
    }
}

void trajectories::computenuevo(Euler &torques)
{
    // Get position, velocity and quaternion from the VRPN object in its coordinate system
    Vector3Df uav_pos, uav_vel; 
    Quaternion vrpn_quaternion;
    uavVrpn->GetPosition(uav_pos);
    uavVrpn->GetSpeed(uav_vel);
    uavVrpn->GetQuaternion(vrpn_quaternion);

    // Get current orientation and angular speed from the AHRS object (IMU)
    const AhrsData *currentOrientation = GetDefaultOrientation();
    Quaternion currentQuaternion;
    Vector3Df currentAngularRates;
    currentOrientation->GetQuaternionAndAngularRates(currentQuaternion, currentAngularRates);
    Vector3Df currentAngularSpeed = GetCurrentAngularSpeed();

    // Use yaw from VRPN and roll, pitch from IMU
    Euler ahrsEuler = currentQuaternion.ToEuler();
    ahrsEuler.yaw = vrpn_quaternion.ToEuler().yaw;
    Quaternion mixQuaternion = ahrsEuler.ToQuaternion();

    // Compute the position and velocity errors in the UAV frame
    Vector2Df pos_error2D, vel_error2D;
    // Example of desired altitude [m] => (ALWAYS A POSITIVE VALUE) 
    // Because the AltitudeValues function returns a positive value also. However, the UAV's altitude is negative in the VRPN coordinate system.
    float yaw_ref;
    float z, dz;

    AltitudeValues(z, dz);
    PositionValues(pos_error2D, vel_error2D, yaw_ref);

    float altittude_desired = desired_position->Value().z;
    float verticalSpeedDesired = 0.0f;

    if (behaviourMode == BehaviourMode_t::StraightLine) {
        altittude_desired = lineAltitudeReference;
        verticalSpeedDesired = lineVerticalSpeed;
    }
    else if (behaviourMode == BehaviourMode_t::PositionHold) {
        altittude_desired = altitudeHold;
    }

    Vector3Df pos_error(
        pos_error2D.x,
        pos_error2D.y,
        z - altittude_desired
    );

    Vector3Df vel_error(
        vel_error2D.x,
        vel_error2D.y,
        dz - verticalSpeedDesired
    );

    // Set the values of the custom controller and update it
    nuevo->SetValues(pos_error, vel_error, mixQuaternion, currentAngularSpeed, yaw_ref);
    nuevo->Update(GetTime());

    // Apply the computed torques and thrust
    torques.roll = nuevo->Output(0);
    torques.pitch = nuevo->Output(1);
    torques.yaw = nuevo->Output(2);
    thrust = nuevo->Output(3); 
    // If you prefer, you can also use the ComputeDefaultThrust() function. E.g.:
    // thrust = ComputeDefaultThrust();
    // The desired take-off altitude will be used as a reference. 
}

float trajectories::ComputeCustomThrust(void)
{
    // Implement your custom thrust computation here or asign its value from another function, because it is a global variable.
    if (thrust == 0)
    {
        // For safety reasons, the default thrust is computed if the custom thrust is not defined.
        thrust = ComputeDefaultThrust();
        std::cout << "Custom thrust not defined, using default thrust: " << thrust << std::endl;
    }
    return thrust;
}

void trajectories::StartCustomTorques(void)
{
    if (control_selection->CurrentIndex() == 0)
    {
        StartDefaultTorques();
        Start_task();
        Thread::Info("trajectories: default control law started\n");
    }
    else
    {
        if(SetTorqueMode(TorqueMode_t::Custom) && SetThrustMode(ThrustMode_t::Custom) && control_selection->CurrentIndex() != 0)
        {
            controlMode_t = ControlMode_t::Custom;
            nuevo->Reset();
            Start_task();
            Thread::Info("trajectories: custom control law started\n");
        }
        else
        {
            StopCustomTorques();
            Thread::Err("trajectories: could not start custom control law\n");
        }
    }
    
}

void trajectories::StopCustomTorques(void)
{
    StartDefaultTorques();
    controlMode_t = ControlMode_t::Default;
    Thread::Info("trajectories: custom control law stopped\n");
}

void trajectories::StartDefaultTorques(void)
{
    if (controlMode_t == ControlMode_t::Default)
    {
        Thread::Warn("trajectories: already in default control law\n");
        return;
    }

    if(SetTorqueMode(TorqueMode_t::Default) && SetThrustMode(ThrustMode_t::Default) )
    {
        controlMode_t = ControlMode_t::Default;
        Thread::Info("trajectories: default control law started\n");
    }
    else
    {
        Thread::Err("trajectories: could not start default control law\n");
    }
}

const AhrsData *trajectories::GetOrientation(void) const {
    //get yaw from vrpn
    Quaternion vrpnQuaternion;
    uavVrpn->GetQuaternion(vrpnQuaternion);

    //get roll, pitch and w from imu
    Quaternion ahrsQuaternion;
    Vector3Df ahrsAngularSpeed;
    GetDefaultOrientation()->GetQuaternionAndAngularRates(ahrsQuaternion, ahrsAngularSpeed);

    // yaw from vrpn and roll, pitch from imu
    Euler ahrsEuler=ahrsQuaternion.ToEuler();
    ahrsEuler.yaw=vrpnQuaternion.ToEuler().yaw;
    Quaternion mixQuaternion=ahrsEuler.ToQuaternion();

    customOrientation->SetQuaternionAndAngularRates(mixQuaternion,ahrsAngularSpeed);

    return customOrientation;
}

void trajectories::AltitudeValues(float &z,float &dz) const{ //'&' allow you to update values from inside the function and still be called outside 
    Vector3Df uav_pos,uav_vel; //Local variables

    uavVrpn->GetPosition(uav_pos);
    uavVrpn->GetSpeed(uav_vel);
    //z and dz must be in uav's frame
    z=-uav_pos.z;
    dz=-uav_vel.z;
}

AhrsData *trajectories::GetReferenceOrientation(void) { //Attitude and Heading Reference System
    Vector2Df pos_err, vel_err; // in Uav coordinate system
    float yaw_ref;
    Euler refAngles;

    PositionValues(pos_err, vel_err, yaw_ref);

    refAngles.yaw=yaw_ref;

    uX->SetValues(pos_err.x, vel_err.x);
    uX->Update(GetTime());
    refAngles.pitch=uX->Output();

    uY->SetValues(pos_err.y, vel_err.y);
    uY->Update(GetTime());
    refAngles.roll=-uY->Output();

    customReferenceOrientation->SetQuaternionAndAngularRates(refAngles.ToQuaternion(),Vector3Df(0,0,0));

    return customReferenceOrientation;
}

void trajectories::PositionValues(Vector2Df &pos_error,Vector2Df &vel_error,float &yaw_ref) {
    Vector3Df uav_pos,uav_vel; // in VRPN coordinate system
    Vector2Df uav_2Dpos,uav_2Dvel; // in VRPN coordinate system

    uavVrpn->GetPosition(uav_pos); //Store current position in uav_pos
    uavVrpn->GetSpeed(uav_vel); //Store current velocity in uav_vel 

    uav_pos.To2Dxy(uav_2Dpos);
    uav_vel.To2Dxy(uav_2Dvel);

    if (behaviourMode==BehaviourMode_t::PositionHold) {
        pos_error=uav_2Dpos-posHold;
        vel_error=uav_2Dvel;
        yaw_ref=yawHold;
    } 
    else if (behaviourMode==BehaviourMode_t::Hover) {
        pos_error=uav_2Dpos;
        vel_error=uav_2Dvel;
        yaw_ref=0;
    } 
    else if (behaviourMode==BehaviourMode_t::Regulation) {
        Vector2Df desired_position_xy(desired_position->Value().x, desired_position->Value().y);
        pos_error=uav_2Dpos - desired_position_xy;
        vel_error=uav_2Dvel;
        yaw_ref=(float)desired_yaw->Value();
    }
    else if (behaviourMode == BehaviourMode_t::StraightLine) {
        double currentTime = static_cast<double>(GetTime()) / 1000000000.0;
        double totalElapsed = currentTime - lineStartTime; //Total seconds the task has been running
        double cycleDuration = 2.0 * lineDuration; //Duration of one round trip
        float elapsed = static_cast<float>(
            std::fmod(totalElapsed, cycleDuration) //Calculates remainder. Tells how far into the current cycle we are
        ); //Converts the remainder from double to float

        bool returning = elapsed >= lineDuration; //True when the travel time has finished

        float legElapsed = elapsed;
        if (returning) {
            legElapsed = elapsed - lineDuration; //Value of 0 when first leg completed. Start of return
        }

        float s = legElapsed / lineDuration; //Fraction of the journey 
        if (s < 0.0f) s = 0.0f;
        if (s > 1.0f) s = 1.0f; //Keeps progress fro each leg

        float progress = 3.0f * s * s - 2.0f * s * s * s; // Position. How much distance should be covered
        float speed = lineDistance * 6.0f * s * (1.0f - s) / lineDuration; //Time derivative of the position

        float referenceX = lineStart.x + lineDistance * progress; //Calculates position in x
        float referenceVx = speed; //We store it to make it negative later

        if (returning) {
            referenceX = lineStart.x + lineDistance * (1.0f - progress); //Decreases from 1 to 0
            referenceVx = -speed;
        }

        Vector2Df referencePosition(referenceX, lineStart.y);
        Vector2Df referenceVelocity(referenceVx, 0.0f);

        float pathProgress = returning ? (1.0f - progress) : progress;//Desired position along te path
        //if=? , else = : , pathProgress increases from 0 to 1 or decreases from 1 to 0
        float progressRate = 6.0f * s * (1.0f - s) / lineDuration; //Fractions of the path per second
        //It's not velocity because it's not multiplied by a distance
        if (returning) {
            progressRate = -progressRate;
        }

        float phase = 2.0f * M_PI * pathProgress;//Converts path progress into the sine wave's angle

        lineAltitudeReference = lineAltitude + altitudeAmplitude * std::sin(phase); //Altitude formula
        lineVerticalSpeed = altitudeAmplitude * 2.0f * M_PI * std::cos(phase) * progressRate; //Differentiate the altitude formula with respect to time

        pos_error = uav_2Dpos - referencePosition;
        vel_error = uav_2Dvel - referenceVelocity;
        yaw_ref = lineYaw;
    }
    else { //Circle
        Vector3Df target_pos;
        Vector2Df circle_pos,circle_vel;
        Vector2Df target_2Dpos;

        targetVrpn->GetPosition(target_pos);
        target_pos.To2Dxy(target_2Dpos);
        circle->SetCenter(target_2Dpos);

        //circle reference
        circle->Update(GetTime()); //provides current time
        circle->GetPosition(circle_pos); // receives where the drone should be now
        circle->GetSpeed(circle_vel); //receives how fast and in which direction

        //error in optitrack frame
        pos_error=uav_2Dpos-circle_pos;
        vel_error=uav_2Dvel-circle_vel;
        yaw_ref=atan2(target_pos.y-uav_pos.y,target_pos.x-uav_pos.x);
    }

    //error in uav frame
    Quaternion currentQuaternion=GetCurrentQuaternion();
    Euler currentAngles;//in vrpn frame
    currentQuaternion.ToEuler(currentAngles);
    pos_error.Rotate(-currentAngles.yaw);
    vel_error.Rotate(-currentAngles.yaw);
}

void trajectories::SignalEvent(Event_t event) {
    UavStateMachine::SignalEvent(event);
    switch(event) {
    case Event_t::TakingOff:
        behaviourMode=BehaviourMode_t::Default;
        vrpnLost=false;
        break;
    case Event_t::EnteringControlLoop:
        if ((behaviourMode==BehaviourMode_t::Circle) && (!circle->IsRunning())) {
            VrpnPositionHold();
        }
        break;
    case Event_t::EnteringFailSafeMode:
        behaviourMode=BehaviourMode_t::Default;
        break;
    }
}

void trajectories::ExtraSecurityCheck(void) {
    if ((!vrpnLost) && ((behaviourMode==BehaviourMode_t::Circle) || (behaviourMode==BehaviourMode_t::PositionHold))) {
        if (!targetVrpn->IsTracked(500)) {
            Thread::Err("VRPN, target lost\n");
            vrpnLost=true;
            EnterFailSafeMode();
            Land();
        }
        if (!uavVrpn->IsTracked(500)) {
            Thread::Err("VRPN, uav lost\n");
            vrpnLost=true;
            EnterFailSafeMode();
            Land();
        }
    }
    if ((!vrpnLost) && ((behaviourMode==BehaviourMode_t::Hover) || (behaviourMode==BehaviourMode_t::Regulation) || (behaviourMode == BehaviourMode_t::StraightLine))) {
        if (!uavVrpn->IsTracked(500)) {
            Thread::Err("VRPN, uav lost\n");
            vrpnLost=true;
            EnterFailSafeMode();
            Land();
        }
    }
}

void trajectories::ExtraCheckPushButton(void) {
    if(startCircle->Clicked()) {
        Start_task();
    }
    if(stopCircle->Clicked()) {
        StopCircle();
    }
    if(positionHold->Clicked()) {
        VrpnPositionHold();
    }
    if(on_customController->Clicked()) {
        StartCustomTorques();
    } 
    if(off_customController->Clicked()) {
        StopCustomTorques();
    }
}

void trajectories::ExtraCheckJoystick(void) {
    /*     Do not use cross, start nor select buttons!!
    0: "start"       1: "select"      2: "square"      3: "triangle"
    4: "circle"      5: "cross";      6: "left 1"      7: "left 2"
    8: "left 3"      9: "right 1"     10: "right 2"    11: "right 3"
    12: "up"         13: "down"       14: "left"       15: "right"
    */

    //R1 and Circle
    if(GetTargetController()->ButtonClicked(4) && GetTargetController()->IsButtonPressed(9)) {
        Start_task();
    }

    //R1 and Cross
    if(GetTargetController()->ButtonClicked(5) && GetTargetController()->IsButtonPressed(9)) {
        StopCircle();
    }
    
    //R1 and Square
    if(GetTargetController()->ButtonClicked(2) && GetTargetController()->IsButtonPressed(9)) {
        VrpnPositionHold();
    }
}

void trajectories::Start_task(void) { //class::function
    if( behaviourMode==BehaviourMode_t::Circle) {
        Thread::Warn("trajectories: already in circle mode\n");
        return;
    }
    if (SetOrientationMode(OrientationMode_t::Custom)) {
        Thread::Info("trajectories: start circle\n");
    } else {
        Thread::Warn("trajectories: could not start circle\n");
        return;
    }

    // Defining desired task. 
    if (task_selection->CurrentIndex() == 0) {
        behaviourMode=BehaviourMode_t::Hover;
        Thread::Info("trajectories: hovering at zero\n");
    }
    else if (task_selection->CurrentIndex() == 1) {
        behaviourMode=BehaviourMode_t::Regulation;
        Thread::Info("trajectories: regulation task\n");
    }
    else if (task_selection->CurrentIndex() == 2) {
        Vector3Df uav_pos,target_pos;
        Vector2Df uav_2Dpos,target_2Dpos;

        targetVrpn->GetPosition(target_pos);
        target_pos.To2Dxy(target_2Dpos);
        circle->SetCenter(target_2Dpos);

        uavVrpn->GetPosition(uav_pos);
        uav_pos.To2Dxy(uav_2Dpos);
        circle->StartTraj(uav_2Dpos);

        uX->Reset();
        uY->Reset();
        behaviourMode=BehaviourMode_t::Circle;
        Thread::Info("trajectories: circle tracking\n");
    }
    else if (task_selection->CurrentIndex() == 3) { //Index 3 because its position in task_selection
        Vector3Df currentPosition; // local variable to store drone's x,y,z
        Quaternion currentQuaternion;

        uavVrpn->GetPosition(currentPosition); // Reads drone's current position from VRPN and stores it in "current Position"
        uavVrpn->GetQuaternion(currentQuaternion); // Reads the drone orientation. '->' calls a function on the object pointed to

        lineStart = Vector2Df(currentPosition.x, currentPosition.y); //Copies current x and y coordinates
        lineAltitude = -currentPosition.z; //Store starting attitude. The project uses negative VRPN z
        lineAltitudeReference = lineAltitude; 
        lineVerticalSpeed = 0.0f; //Initial desired vertical speed
        lineYaw = currentQuaternion.ToEuler().yaw; //Convert into Euler and assign yaw
        lineStartTime = static_cast<double>(GetTime()) / 1000000000.0; //GetTime uses nanoseconds
        //Records when the task starts
        uX->Reset(); //Start task fresh
        uY->Reset();

        behaviourMode = BehaviourMode_t::StraightLine; //Changes the flight task to Straight linr
        Thread::Info("trajectories: straight-line tracking started\n"); //Informational message
    }
    else {
        Thread::Err("trajectories: unknown task\n");
        return;
    }
}

void trajectories::StopCircle(void) {
    if (behaviourMode == BehaviourMode_t::StraightLine) {
        VrpnPositionHold(); //Switches to holding the drone's current position
        Thread::Info("trajectories: straight line stopped, holding position\n");
        return;
    }
    if( behaviourMode!=BehaviourMode_t::Circle) {
        Thread::Warn("trajectories: not in circle mode\n");
        return;
    }
    circle->FinishTraj();
    //GetJoystick()->Rumble(0x70);
    Thread::Info("trajectories: finishing circle\n");
}

void trajectories::VrpnPositionHold(void) {
    if( behaviourMode==BehaviourMode_t::PositionHold) {
        Thread::Warn("trajectories: already in vrpn position hold mode\n");
        return;
    }
	Quaternion vrpnQuaternion;
    uavVrpn->GetQuaternion(vrpnQuaternion);
	yawHold=vrpnQuaternion.ToEuler().yaw;

    Vector3Df vrpnPosition;
    uavVrpn->GetPosition(vrpnPosition);
    vrpnPosition.To2Dxy(posHold);
    altitudeHold = -vrpnPosition.z; //Captures current altitude when position hold starts

    uX->Reset();
    uY->Reset();
    behaviourMode=BehaviourMode_t::PositionHold;
    SetOrientationMode(OrientationMode_t::Custom);
    Thread::Info("trajectories: holding position\n");
}
