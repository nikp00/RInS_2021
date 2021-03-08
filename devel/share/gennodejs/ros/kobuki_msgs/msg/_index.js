
"use strict";

let WheelDropEvent = require('./WheelDropEvent.js');
let PowerSystemEvent = require('./PowerSystemEvent.js');
let MotorPower = require('./MotorPower.js');
let ScanAngle = require('./ScanAngle.js');
let DigitalOutput = require('./DigitalOutput.js');
let Led = require('./Led.js');
let CliffEvent = require('./CliffEvent.js');
let DigitalInputEvent = require('./DigitalInputEvent.js');
let DockInfraRed = require('./DockInfraRed.js');
let ControllerInfo = require('./ControllerInfo.js');
let KeyboardInput = require('./KeyboardInput.js');
let SensorState = require('./SensorState.js');
let VersionInfo = require('./VersionInfo.js');
let BumperEvent = require('./BumperEvent.js');
let ButtonEvent = require('./ButtonEvent.js');
let ExternalPower = require('./ExternalPower.js');
let RobotStateEvent = require('./RobotStateEvent.js');
let Sound = require('./Sound.js');
let AutoDockingActionGoal = require('./AutoDockingActionGoal.js');
let AutoDockingResult = require('./AutoDockingResult.js');
let AutoDockingAction = require('./AutoDockingAction.js');
let AutoDockingFeedback = require('./AutoDockingFeedback.js');
let AutoDockingActionFeedback = require('./AutoDockingActionFeedback.js');
let AutoDockingActionResult = require('./AutoDockingActionResult.js');
let AutoDockingGoal = require('./AutoDockingGoal.js');

module.exports = {
  WheelDropEvent: WheelDropEvent,
  PowerSystemEvent: PowerSystemEvent,
  MotorPower: MotorPower,
  ScanAngle: ScanAngle,
  DigitalOutput: DigitalOutput,
  Led: Led,
  CliffEvent: CliffEvent,
  DigitalInputEvent: DigitalInputEvent,
  DockInfraRed: DockInfraRed,
  ControllerInfo: ControllerInfo,
  KeyboardInput: KeyboardInput,
  SensorState: SensorState,
  VersionInfo: VersionInfo,
  BumperEvent: BumperEvent,
  ButtonEvent: ButtonEvent,
  ExternalPower: ExternalPower,
  RobotStateEvent: RobotStateEvent,
  Sound: Sound,
  AutoDockingActionGoal: AutoDockingActionGoal,
  AutoDockingResult: AutoDockingResult,
  AutoDockingAction: AutoDockingAction,
  AutoDockingFeedback: AutoDockingFeedback,
  AutoDockingActionFeedback: AutoDockingActionFeedback,
  AutoDockingActionResult: AutoDockingActionResult,
  AutoDockingGoal: AutoDockingGoal,
};
