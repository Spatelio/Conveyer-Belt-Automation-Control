# **Conveyor Belt Control System**



**Author:** Sascha Patel



A PLC program written Structured Text (ST) using CODESYS V3.5, implementing a full conveyor belt speed control system with PID motor control, a 5-state machine, and a tiered alarm management system with debounced fault detection.



## **Features**

* closed-loop PID speed control with anti-windup clamping
* 5-state machine: IDLE → RUN → WARN → FAULT → ESTOP
* debounced fault detection using TON timers
* latched alarm logic -> faults stay active until operator explicitly resets
* tiered alarm severity -> warning vs fault vs estop handled independently based on severity
* clean FB architecture -> reusable, independently testable function blocks
* fully simulated and tested on soft PLC





## **Getting Started**



### Running the Project

1. clone or download
2. open `ConveyorBeltControl.project` in CODESYS
3. start soft PLC runtime from taskbar tray
4. in the IDE: double-click Device → Scan Network → local PLC
5. press Alt+F8 to login → download → F5 to run
6. open Debug → add GVL\_Conveyor variables to monitor live values



### Simulating Inputs

With project running, use Watch Window to force variable values:

* click value column next to any variable
* type a new value and press Enter
* PLC picks it up on the next scan cycle (within 10ms)

\---





## **Testing**

Seven tests were performed on soft PLC, forcing inputs and observing outputs in real time to test full operator workflow and faults.



|**TEST**|**VALUES FORCED**|**EXPECTED OUTPUT OBSERVED**|
|-|-|-|
|**IDLE**|none - baseline checked|eState = 0<br />xMotorEnable = FALSE<br />rMotorOuput = 0|
|**STARTUP SETPOINT**|rSpeedSetpoint -> 100|belt ramps from 0 to 100 RPM<br />PID output reduces with belt speed|
|**MID-RUN SETPOINT INCREASE**|rSpeedSetpoint = 50 initially,<br />then rSpeedSetpoint -> 100|belt tracks change smoothly<br />no overshooting or noticeable oscillation|
|**MID-RUN SETPOINT INCREASE**|rSpeedSetpoint = 100 initially,<br />then rSpeedSetpoint -> 50|belt tracks change smoothly<br />|
|**UNDERSPEED - HEAVY LOAD**|fbBeltSim.rAccelRate → 0.02,<br />rSpeedSetpoint → 100|belt falls behind ramp<br />threshold tracks ramp<br />after 3s eState=2 WARN<br />belt still running|
|**FAULT LATCH CHECK**|xOverloadRelay still TRUE,<br />xResetRequest → TRUE|stays in FAULT<br />latch held while physical fault still present|
|**FAULT RECOVERY**|xOverloadRelay → FALSE,<br />xResetRequest → TRUE then FALSE|eState=0 IDLE<br />xFaultLight=FALSE<br />rRampedSetpoint reset to 0|
|**ESTOP FROM RUN**|get to RUN,<br />then xEStop → TRUE|eState=4 instantly<br />xMotorEnable=FALSE<br />rMotorOutput=0<br />no timer delay|
|**ESTOP HELD**|xEStop still TRUE,<br />xResetRequest → TRUE|stays in ESTOP<br />cannot reset while button physically held|
|**ESTOP RECOVERY**|xEStop → FALSE,<br />xResetRequest → TRUE then FALSE|eState=0 IDLE<br />xAlarm\_EStop=FALSE<br />system ready to restart|
|**ANTI-WINDUP VERIFICATION**|fbBeltSim.rAccelRate → 0.0 (stalled belt),<br />rSpeedSetpoint=100|rMotorOutput=100%<br />xLimitActive=TRUE<br />rIntegral plateaus and stops growing|
|**ANTI-WINDUP RECOVERY**|fbBeltSim.rAccelRate → 0.5 (belt freed)|rActualSpeed ramps up smoothly, no violent overshoot, rMotorOutput reduces gradually|



More testing was preformed to verify the general working behaviour of the system and state transitions. More testing could be preformed to further tune control parameters to desired belt behaviour.





## **Project Structure**

```
ConveyorBeltControl/
├── GVL\\\_Conveyor        global variable list — all I/O and status signals
├── FB\\\_PIDController    PID function block with anti-windup
├── FB\\\_AlarmManager     alarm detection, debounce, latching, and reset logic

├── FB\\\_BeltSimulator    real conveyer belt simulation to emulate speed output with real conditions (closed loop system)
└── PRG\\\_Main            state machine — orchestrates all FBs and I/O
```







## **Architecture**

```
        INPUTS                  PLC PROGRAM                   OUTPUTS
   ┌──────────────┐         ┌────────────────────┐       ┌──────────────┐
   │ speed sensor │────────►│   state machine    │──────►│ motor drive  │
   │  actual RPM  │         │   IDLE/RUN/WARN/   │       │  0-100% cmd  │
   └──────────────┘         │    FAULT/ESTOP     │       └──────────────┘
   ┌──────────────┐         ├────────────────────┤       ┌──────────────┐
   │   estop btn  │────────►│  FB\\\_PIDController │──────►│ HMI display  │
   │  digital in  │         │    Kp · Ki · Kd    │       │ speed/state  │
   └──────────────┘         ├────────────────────┤       └──────────────┘
   ┌──────────────┐         │  FB\\\_AlarmManager  │       ┌──────────────┐
   │ overload rly │────────►│   TON debounce     │──────►│ alarm beacon │
   │ motor current│         │   latch · reset    │       │ warn / fault │
   └──────────────┘         ├────────────────────┤       └──────────────┘
   ┌──────────────┐         │    GVL\\\_Conveyor   │       ┌──────────────┐
   │speed setpoint│────────►│   global I/O bus   │──────►│  event log   │
   │    op input  │         │                    │       │  timestamps  │
   └──────────────┘         └────────────────────┘       └──────────────┘
```





## **State Machine**

The conveyor operates as a deterministic 5-state machine where every possible system condition
maps to exactly one state with clear transitions.

```
                 setpoint > 0
                 no alarms active
         ┌─────────────────────────────┐
         │                             ▼
      ┌──┴───┐    start cmd       ┌─────────┐
      │ **IDLE** │───────────────────►│   **RUN**   │
      │ (0)  │                    │   (1)   │
      └──────┘                    └────┬────┘
         ▲                             │ any alarm active
         │                             ▼
         │                        ┌─────────┐
         │                        │  **WARN**   │ belt still running
         │                        │   (2)   │ underspeed only
         │                        └────┬────┘
         │                             │ overload or estop confirmed
         │    reset request            ▼
         └───────────────────────┌─────────┐
                                 │  **FAULT**  │ motor stopped
                                 │   (3)   │ needs operator reset
                                 └─────────┘

         ┌──────────────────────────────────────────────┐
         │      **E-STOP** (4) - triggered from ANY state   │
         │             - immediate motor cut            │
         │      clears only when: xEStop = FALSE        │
         │             AND  xResetRequest = TRUE        │
         └──────────────────────────────────────────────┘
```

### State Transition table

|**FROM STATE**|**CONDITION**|**TO STATE**|
|-|-|-|
|**IDLE**|setpoint > 0, no alarms|**RUN**|
|**RUN**|any alarm active|**WARN**|
|**RUN**|setpoint = 0|**IDLE**|
|**WARN**|overload or estop confirmed|**FAULT**|
|**WARN**|all alarms cleared|**RUN**|
|**FAULT**|reset request + no active faults|**IDLE**|
|**ANY**|xEStop = TRUE|**ESTOP**|
|**ESTOP**|xEStop = FALSE + reset request|**IDLE**|

\---

## 

## **PID Controller**

Standard positional PID algorithm with output clamping and anti-windup.

```
\*\*error\*\* = setpoint − actual\\\\\\\_speed

\*\*P term\*\* = Kp × error
             reacts to current error magnitude

\*\*I term\*\* = Ki × Σ(error × dt)
             eliminates steady-state error over time
             dt = 0.01s (10ms cycle time)

\*\*D term\*\* = Kd × (error − previous\\\\\\\_error)
             dampens overshoot by reacting to rate of change

\*\*output\*\* = P + I + D   clamped to \\\\\\\[rOutputMin, rOutputMax]
```

### Anti-windup

When the output hits clamp limit, the integral accumulation is reversed by one step to prevent the integrator from growing without a bound during saturation to prevent severe overshoot when the limit condition clears.

```
if rRawOutput > rOutputMax:
    rOutput   = rOutputMax
    rIntegral = rIntegral - (rError × dt)   ← undo last accumulation
```

### Tuning Parameters (starting values)

|**parameter**|**value**|**effect**|
|-|-|-|
|Kp|1.2|primary correction strength|
|Ki|0.4|steady-state error elimination speed|
|Kd|0.05|overshoot damping — kept low for conveyors|

\---

## 

## **Alarm management**



### Fault Types and Severity

```
┌─────────────────┬──────────┬─────────┬──────────────────────────────┐
│ fault           │ severity │  delay  │ behaviour                    │
├─────────────────┼──────────┼─────────┼──────────────────────────────┤
│ E-stop          │ CRITICAL │  none   │ immediate latch, motor cut   │
│ overload relay  │ FAULT    │  2s     │ TON debounce, motor cut      │
│ underspeed      │ WARNING  │  3s     │ TON debounce, belt continues │
└─────────────────┴──────────┴─────────┴──────────────────────────────┘
```

### Debounce Timing, Latch \& Reset Behaviour

Physical sensors can flicker for milliseconds due to vibration and electrical noise. To avoid false positives, TON timers confirm the fault condition has been sustained before latching.



Once latched, an alarm stays active even if the physical fault clears. Faults are not cleared until an operator explicitly confirms safety manually.

```
fault occurs   ──► alarm latches TRUE
fault clears   ──► alarm stays TRUE
operator reset ──► alarm clears
```

### Underspeed Threshold

```
rUnderspeedThreshold = rSpeedSetpoint × 0.70

belt running at 100 RPM setpoint:
  threshold = 70 RPM
  if actual < 70 RPM for > 3s → underspeed alarm latches
```

\---



## **Belt Simulator**



To simulate real response from a deployed belt, we model physical behaviour of a belt under load.



This is achieved through a few simulation parameters:



**rAccelRate = 0.5 (defaults)**	RPM gained per scan - lower value simulates heavier load, slower response to motor output

**rFriction  = 0.3**		RPM lost per scan when motor is off

**rMaxSpeed  = 150.0**		RPM at 100% motor output (we cap at 100RPM, correlating to \~66% motor output)





## **Global Variable List**

## naming convention

Variables follow hungarian notation standard as close as possible to keep consistency:

```
x  = BOOL    xEStop, xMotorEnable
r  = REAL    rActualSpeed, rMotorOutput
e  = INT     eConveyorState
t  = TIME    tDelayTimer
fb = FB      fbPID, fbAlarms
```

### Variables

```
// inputs
\*\*xEStop\*\*              BOOL    true = emergency stop pressed
\*\*xOverloadRelay\*\*      BOOL    true = motor drawing excess current
\*\*rActualSpeed\*\*        REAL    belt speed from sensor in RPM
\*\*rSpeedSetpoint\*\*      REAL    desired speed from HMI operator

// outputs
\*\*rMotorOutput\*\*        REAL    0.0 to 100.0 — drives the VFD
\*\*xMotorEnable\*\*        BOOL    true = motor contactor closed
\*\*xAlarmLight\*\*         BOOL    type 1 beacon — any alarm active
\*\*xFaultLight\*\*         BOOL    type 2 beacon — fault or estop only

// status
\*\*eConveyorState\*\*      INT     0=IDLE 1=RUN 2=WARN 3=FAULT 4=ESTOP
\*\*xSystemReady\*\*        BOOL    true = no faults safe to start
\*\*xResetRequest\*\*       BOOL    operator reset from HMI

// alarm flags
\*\*xAlarm\\\\\\\_EStop\*\*       BOOL   latched estop alarm
\*\*xAlarm\\\\\\\_Overload\*\*    BOOL   latched overload alarm
\*\*xAlarm\\\\\\\_Underspeed\*\*  BOOL   latched underspeed alarm
```

\---

