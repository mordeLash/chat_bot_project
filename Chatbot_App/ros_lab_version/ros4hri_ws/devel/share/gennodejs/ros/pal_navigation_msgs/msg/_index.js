
"use strict";

let JoyPriorityActionGoal = require('./JoyPriorityActionGoal.js');
let JoyPriorityActionResult = require('./JoyPriorityActionResult.js');
let VisualTrainingFeedback = require('./VisualTrainingFeedback.js');
let ExecuteParkingActionResult = require('./ExecuteParkingActionResult.js');
let JoyTurboAction = require('./JoyTurboAction.js');
let GoToActionGoal = require('./GoToActionGoal.js');
let ExecuteParkingGoal = require('./ExecuteParkingGoal.js');
let GoToPOIGoal = require('./GoToPOIGoal.js');
let FollowWaypointsGoal = require('./FollowWaypointsGoal.js');
let ExecuteParkingActionGoal = require('./ExecuteParkingActionGoal.js');
let FollowWaypointsFeedback = require('./FollowWaypointsFeedback.js');
let FollowWaypointsActionResult = require('./FollowWaypointsActionResult.js');
let GoToActionResult = require('./GoToActionResult.js');
let JoyTurboFeedback = require('./JoyTurboFeedback.js');
let GoToAction = require('./GoToAction.js');
let JoyTurboResult = require('./JoyTurboResult.js');
let VisualTrainingActionResult = require('./VisualTrainingActionResult.js');
let VisualTrainingResult = require('./VisualTrainingResult.js');
let JoyPriorityAction = require('./JoyPriorityAction.js');
let JoyPriorityResult = require('./JoyPriorityResult.js');
let VisualTrainingAction = require('./VisualTrainingAction.js');
let VisualTrainingGoal = require('./VisualTrainingGoal.js');
let GoToPOIActionGoal = require('./GoToPOIActionGoal.js');
let GoToFeedback = require('./GoToFeedback.js');
let JoyTurboActionGoal = require('./JoyTurboActionGoal.js');
let GoToPOIFeedback = require('./GoToPOIFeedback.js');
let JoyTurboGoal = require('./JoyTurboGoal.js');
let GoToResult = require('./GoToResult.js');
let JoyPriorityActionFeedback = require('./JoyPriorityActionFeedback.js');
let ExecuteParkingResult = require('./ExecuteParkingResult.js');
let GoToPOIActionResult = require('./GoToPOIActionResult.js');
let VisualTrainingActionFeedback = require('./VisualTrainingActionFeedback.js');
let JoyPriorityFeedback = require('./JoyPriorityFeedback.js');
let GoToActionFeedback = require('./GoToActionFeedback.js');
let ExecuteParkingFeedback = require('./ExecuteParkingFeedback.js');
let GoToPOIActionFeedback = require('./GoToPOIActionFeedback.js');
let FollowWaypointsResult = require('./FollowWaypointsResult.js');
let JoyTurboActionResult = require('./JoyTurboActionResult.js');
let FollowWaypointsAction = require('./FollowWaypointsAction.js');
let ExecuteParkingAction = require('./ExecuteParkingAction.js');
let JoyPriorityGoal = require('./JoyPriorityGoal.js');
let FollowWaypointsActionFeedback = require('./FollowWaypointsActionFeedback.js');
let GoToGoal = require('./GoToGoal.js');
let GoToPOIAction = require('./GoToPOIAction.js');
let ExecuteParkingActionFeedback = require('./ExecuteParkingActionFeedback.js');
let JoyTurboActionFeedback = require('./JoyTurboActionFeedback.js');
let VisualTrainingActionGoal = require('./VisualTrainingActionGoal.js');
let FollowWaypointsActionGoal = require('./FollowWaypointsActionGoal.js');
let GoToPOIResult = require('./GoToPOIResult.js');
let MapConfiguration = require('./MapConfiguration.js');
let ServiceStatus = require('./ServiceStatus.js');
let PolarReadingScan = require('./PolarReadingScan.js');
let AvailableMaps = require('./AvailableMaps.js');
let POIGroup = require('./POIGroup.js');
let TabletPOI = require('./TabletPOI.js');
let LaserImage = require('./LaserImage.js');
let NavigationStatus = require('./NavigationStatus.js');
let EulerAnglesStamped = require('./EulerAnglesStamped.js');
let MissedWaypoint = require('./MissedWaypoint.js');
let Highways = require('./Highways.js');
let Waypoint = require('./Waypoint.js');
let EulerAngles = require('./EulerAngles.js');
let NiceMapTransformation = require('./NiceMapTransformation.js');
let VisualLocDB = require('./VisualLocDB.js');
let POI = require('./POI.js');
let PolarReading = require('./PolarReading.js');
let Emergency = require('./Emergency.js');

module.exports = {
  JoyPriorityActionGoal: JoyPriorityActionGoal,
  JoyPriorityActionResult: JoyPriorityActionResult,
  VisualTrainingFeedback: VisualTrainingFeedback,
  ExecuteParkingActionResult: ExecuteParkingActionResult,
  JoyTurboAction: JoyTurboAction,
  GoToActionGoal: GoToActionGoal,
  ExecuteParkingGoal: ExecuteParkingGoal,
  GoToPOIGoal: GoToPOIGoal,
  FollowWaypointsGoal: FollowWaypointsGoal,
  ExecuteParkingActionGoal: ExecuteParkingActionGoal,
  FollowWaypointsFeedback: FollowWaypointsFeedback,
  FollowWaypointsActionResult: FollowWaypointsActionResult,
  GoToActionResult: GoToActionResult,
  JoyTurboFeedback: JoyTurboFeedback,
  GoToAction: GoToAction,
  JoyTurboResult: JoyTurboResult,
  VisualTrainingActionResult: VisualTrainingActionResult,
  VisualTrainingResult: VisualTrainingResult,
  JoyPriorityAction: JoyPriorityAction,
  JoyPriorityResult: JoyPriorityResult,
  VisualTrainingAction: VisualTrainingAction,
  VisualTrainingGoal: VisualTrainingGoal,
  GoToPOIActionGoal: GoToPOIActionGoal,
  GoToFeedback: GoToFeedback,
  JoyTurboActionGoal: JoyTurboActionGoal,
  GoToPOIFeedback: GoToPOIFeedback,
  JoyTurboGoal: JoyTurboGoal,
  GoToResult: GoToResult,
  JoyPriorityActionFeedback: JoyPriorityActionFeedback,
  ExecuteParkingResult: ExecuteParkingResult,
  GoToPOIActionResult: GoToPOIActionResult,
  VisualTrainingActionFeedback: VisualTrainingActionFeedback,
  JoyPriorityFeedback: JoyPriorityFeedback,
  GoToActionFeedback: GoToActionFeedback,
  ExecuteParkingFeedback: ExecuteParkingFeedback,
  GoToPOIActionFeedback: GoToPOIActionFeedback,
  FollowWaypointsResult: FollowWaypointsResult,
  JoyTurboActionResult: JoyTurboActionResult,
  FollowWaypointsAction: FollowWaypointsAction,
  ExecuteParkingAction: ExecuteParkingAction,
  JoyPriorityGoal: JoyPriorityGoal,
  FollowWaypointsActionFeedback: FollowWaypointsActionFeedback,
  GoToGoal: GoToGoal,
  GoToPOIAction: GoToPOIAction,
  ExecuteParkingActionFeedback: ExecuteParkingActionFeedback,
  JoyTurboActionFeedback: JoyTurboActionFeedback,
  VisualTrainingActionGoal: VisualTrainingActionGoal,
  FollowWaypointsActionGoal: FollowWaypointsActionGoal,
  GoToPOIResult: GoToPOIResult,
  MapConfiguration: MapConfiguration,
  ServiceStatus: ServiceStatus,
  PolarReadingScan: PolarReadingScan,
  AvailableMaps: AvailableMaps,
  POIGroup: POIGroup,
  TabletPOI: TabletPOI,
  LaserImage: LaserImage,
  NavigationStatus: NavigationStatus,
  EulerAnglesStamped: EulerAnglesStamped,
  MissedWaypoint: MissedWaypoint,
  Highways: Highways,
  Waypoint: Waypoint,
  EulerAngles: EulerAngles,
  NiceMapTransformation: NiceMapTransformation,
  VisualLocDB: VisualLocDB,
  POI: POI,
  PolarReading: PolarReading,
  Emergency: Emergency,
};
