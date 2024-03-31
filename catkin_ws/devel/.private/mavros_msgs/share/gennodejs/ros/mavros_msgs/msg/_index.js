
"use strict";

let HilControls = require('./HilControls.js');
let ESCTelemetry = require('./ESCTelemetry.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let MountControl = require('./MountControl.js');
let AttitudeTarget = require('./AttitudeTarget.js');
let VFR_HUD = require('./VFR_HUD.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let CommandCode = require('./CommandCode.js');
let DebugValue = require('./DebugValue.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let CameraImageCaptured = require('./CameraImageCaptured.js');
let RTKBaseline = require('./RTKBaseline.js');
let TerrainReport = require('./TerrainReport.js');
let StatusText = require('./StatusText.js');
let ManualControl = require('./ManualControl.js');
let Altitude = require('./Altitude.js');
let RadioStatus = require('./RadioStatus.js');
let State = require('./State.js');
let RCOut = require('./RCOut.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let GPSRTK = require('./GPSRTK.js');
let Param = require('./Param.js');
let RTCM = require('./RTCM.js');
let ESCTelemetryItem = require('./ESCTelemetryItem.js');
let RCIn = require('./RCIn.js');
let CompanionProcessStatus = require('./CompanionProcessStatus.js');
let PlayTuneV2 = require('./PlayTuneV2.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let WaypointReached = require('./WaypointReached.js');
let MagnetometerReporter = require('./MagnetometerReporter.js');
let HilGPS = require('./HilGPS.js');
let FileEntry = require('./FileEntry.js');
let Trajectory = require('./Trajectory.js');
let HomePosition = require('./HomePosition.js');
let ESCStatusItem = require('./ESCStatusItem.js');
let Tunnel = require('./Tunnel.js');
let LandingTarget = require('./LandingTarget.js');
let WaypointList = require('./WaypointList.js');
let LogData = require('./LogData.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let EstimatorStatus = require('./EstimatorStatus.js');
let ActuatorControl = require('./ActuatorControl.js');
let BatteryStatus = require('./BatteryStatus.js');
let Mavlink = require('./Mavlink.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let VehicleInfo = require('./VehicleInfo.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');
let HilSensor = require('./HilSensor.js');
let ExtendedState = require('./ExtendedState.js');
let NavControllerOutput = require('./NavControllerOutput.js');
let GPSINPUT = require('./GPSINPUT.js');
let ParamValue = require('./ParamValue.js');
let LogEntry = require('./LogEntry.js');
let ESCInfoItem = require('./ESCInfoItem.js');
let Thrust = require('./Thrust.js');
let ESCInfo = require('./ESCInfo.js');
let ESCStatus = require('./ESCStatus.js');
let GPSRAW = require('./GPSRAW.js');
let Vibration = require('./Vibration.js');
let Waypoint = require('./Waypoint.js');
let PositionTarget = require('./PositionTarget.js');

module.exports = {
  HilControls: HilControls,
  ESCTelemetry: ESCTelemetry,
  OpticalFlowRad: OpticalFlowRad,
  MountControl: MountControl,
  AttitudeTarget: AttitudeTarget,
  VFR_HUD: VFR_HUD,
  GlobalPositionTarget: GlobalPositionTarget,
  CommandCode: CommandCode,
  DebugValue: DebugValue,
  TimesyncStatus: TimesyncStatus,
  CameraImageCaptured: CameraImageCaptured,
  RTKBaseline: RTKBaseline,
  TerrainReport: TerrainReport,
  StatusText: StatusText,
  ManualControl: ManualControl,
  Altitude: Altitude,
  RadioStatus: RadioStatus,
  State: State,
  RCOut: RCOut,
  HilStateQuaternion: HilStateQuaternion,
  WheelOdomStamped: WheelOdomStamped,
  GPSRTK: GPSRTK,
  Param: Param,
  RTCM: RTCM,
  ESCTelemetryItem: ESCTelemetryItem,
  RCIn: RCIn,
  CompanionProcessStatus: CompanionProcessStatus,
  PlayTuneV2: PlayTuneV2,
  OverrideRCIn: OverrideRCIn,
  CamIMUStamp: CamIMUStamp,
  WaypointReached: WaypointReached,
  MagnetometerReporter: MagnetometerReporter,
  HilGPS: HilGPS,
  FileEntry: FileEntry,
  Trajectory: Trajectory,
  HomePosition: HomePosition,
  ESCStatusItem: ESCStatusItem,
  Tunnel: Tunnel,
  LandingTarget: LandingTarget,
  WaypointList: WaypointList,
  LogData: LogData,
  HilActuatorControls: HilActuatorControls,
  EstimatorStatus: EstimatorStatus,
  ActuatorControl: ActuatorControl,
  BatteryStatus: BatteryStatus,
  Mavlink: Mavlink,
  ADSBVehicle: ADSBVehicle,
  VehicleInfo: VehicleInfo,
  OnboardComputerStatus: OnboardComputerStatus,
  HilSensor: HilSensor,
  ExtendedState: ExtendedState,
  NavControllerOutput: NavControllerOutput,
  GPSINPUT: GPSINPUT,
  ParamValue: ParamValue,
  LogEntry: LogEntry,
  ESCInfoItem: ESCInfoItem,
  Thrust: Thrust,
  ESCInfo: ESCInfo,
  ESCStatus: ESCStatus,
  GPSRAW: GPSRAW,
  Vibration: Vibration,
  Waypoint: Waypoint,
  PositionTarget: PositionTarget,
};
