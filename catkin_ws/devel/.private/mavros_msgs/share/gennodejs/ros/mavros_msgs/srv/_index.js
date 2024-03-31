
"use strict";

let CommandHome = require('./CommandHome.js')
let FileRename = require('./FileRename.js')
let CommandInt = require('./CommandInt.js')
let LogRequestEnd = require('./LogRequestEnd.js')
let MountConfigure = require('./MountConfigure.js')
let FileMakeDir = require('./FileMakeDir.js')
let ParamPull = require('./ParamPull.js')
let FileList = require('./FileList.js')
let WaypointClear = require('./WaypointClear.js')
let CommandLong = require('./CommandLong.js')
let ParamSet = require('./ParamSet.js')
let CommandVtolTransition = require('./CommandVtolTransition.js')
let FileRemove = require('./FileRemove.js')
let CommandTriggerInterval = require('./CommandTriggerInterval.js')
let StreamRate = require('./StreamRate.js')
let LogRequestList = require('./LogRequestList.js')
let FileClose = require('./FileClose.js')
let SetMavFrame = require('./SetMavFrame.js')
let FileOpen = require('./FileOpen.js')
let FileWrite = require('./FileWrite.js')
let FileChecksum = require('./FileChecksum.js')
let VehicleInfoGet = require('./VehicleInfoGet.js')
let CommandAck = require('./CommandAck.js')
let WaypointPush = require('./WaypointPush.js')
let FileRemoveDir = require('./FileRemoveDir.js')
let ParamPush = require('./ParamPush.js')
let WaypointPull = require('./WaypointPull.js')
let FileTruncate = require('./FileTruncate.js')
let ParamGet = require('./ParamGet.js')
let WaypointSetCurrent = require('./WaypointSetCurrent.js')
let CommandBool = require('./CommandBool.js')
let SetMode = require('./SetMode.js')
let FileRead = require('./FileRead.js')
let CommandTOL = require('./CommandTOL.js')
let MessageInterval = require('./MessageInterval.js')
let LogRequestData = require('./LogRequestData.js')
let CommandTriggerControl = require('./CommandTriggerControl.js')

module.exports = {
  CommandHome: CommandHome,
  FileRename: FileRename,
  CommandInt: CommandInt,
  LogRequestEnd: LogRequestEnd,
  MountConfigure: MountConfigure,
  FileMakeDir: FileMakeDir,
  ParamPull: ParamPull,
  FileList: FileList,
  WaypointClear: WaypointClear,
  CommandLong: CommandLong,
  ParamSet: ParamSet,
  CommandVtolTransition: CommandVtolTransition,
  FileRemove: FileRemove,
  CommandTriggerInterval: CommandTriggerInterval,
  StreamRate: StreamRate,
  LogRequestList: LogRequestList,
  FileClose: FileClose,
  SetMavFrame: SetMavFrame,
  FileOpen: FileOpen,
  FileWrite: FileWrite,
  FileChecksum: FileChecksum,
  VehicleInfoGet: VehicleInfoGet,
  CommandAck: CommandAck,
  WaypointPush: WaypointPush,
  FileRemoveDir: FileRemoveDir,
  ParamPush: ParamPush,
  WaypointPull: WaypointPull,
  FileTruncate: FileTruncate,
  ParamGet: ParamGet,
  WaypointSetCurrent: WaypointSetCurrent,
  CommandBool: CommandBool,
  SetMode: SetMode,
  FileRead: FileRead,
  CommandTOL: CommandTOL,
  MessageInterval: MessageInterval,
  LogRequestData: LogRequestData,
  CommandTriggerControl: CommandTriggerControl,
};
