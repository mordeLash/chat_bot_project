
"use strict";

let BatteryState = require('./BatteryState.js');
let LedFixedColorParams = require('./LedFixedColorParams.js');
let LedProgressParams = require('./LedProgressParams.js');
let Bumper = require('./Bumper.js');
let LedPreProgrammedParams = require('./LedPreProgrammedParams.js');
let LedFadeParams = require('./LedFadeParams.js');
let LedFlowParams = require('./LedFlowParams.js');
let LedDataArrayParams = require('./LedDataArrayParams.js');
let LedEffectParams = require('./LedEffectParams.js');
let LedEffectViaTopicParams = require('./LedEffectViaTopicParams.js');
let LedRainbowParams = require('./LedRainbowParams.js');
let LedGroup = require('./LedGroup.js');
let LedBlinkParams = require('./LedBlinkParams.js');
let DoTimedLedEffectActionResult = require('./DoTimedLedEffectActionResult.js');
let DoTimedLedEffectActionGoal = require('./DoTimedLedEffectActionGoal.js');
let DoTimedLedEffectActionFeedback = require('./DoTimedLedEffectActionFeedback.js');
let DoTimedLedEffectResult = require('./DoTimedLedEffectResult.js');
let DoTimedLedEffectAction = require('./DoTimedLedEffectAction.js');
let DoTimedLedEffectGoal = require('./DoTimedLedEffectGoal.js');
let DoTimedLedEffectFeedback = require('./DoTimedLedEffectFeedback.js');

module.exports = {
  BatteryState: BatteryState,
  LedFixedColorParams: LedFixedColorParams,
  LedProgressParams: LedProgressParams,
  Bumper: Bumper,
  LedPreProgrammedParams: LedPreProgrammedParams,
  LedFadeParams: LedFadeParams,
  LedFlowParams: LedFlowParams,
  LedDataArrayParams: LedDataArrayParams,
  LedEffectParams: LedEffectParams,
  LedEffectViaTopicParams: LedEffectViaTopicParams,
  LedRainbowParams: LedRainbowParams,
  LedGroup: LedGroup,
  LedBlinkParams: LedBlinkParams,
  DoTimedLedEffectActionResult: DoTimedLedEffectActionResult,
  DoTimedLedEffectActionGoal: DoTimedLedEffectActionGoal,
  DoTimedLedEffectActionFeedback: DoTimedLedEffectActionFeedback,
  DoTimedLedEffectResult: DoTimedLedEffectResult,
  DoTimedLedEffectAction: DoTimedLedEffectAction,
  DoTimedLedEffectGoal: DoTimedLedEffectGoal,
  DoTimedLedEffectFeedback: DoTimedLedEffectFeedback,
};
