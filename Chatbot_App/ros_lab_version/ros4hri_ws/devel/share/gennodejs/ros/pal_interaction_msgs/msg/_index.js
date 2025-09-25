
"use strict";

let audiosignal = require('./audiosignal.js');
let asrresult = require('./asrresult.js');
let WebGuiEvent = require('./WebGuiEvent.js');
let AudioPlayerState = require('./AudioPlayerState.js');
let EnablingSoundLocalisation = require('./EnablingSoundLocalisation.js');
let VoiceActivity = require('./VoiceActivity.js');
let asrupdate = require('./asrupdate.js');
let ASRLanguage = require('./ASRLanguage.js');
let InputArgument = require('./InputArgument.js');
let ASREvent = require('./ASREvent.js');
let ASRActivation = require('./ASRActivation.js');
let TtsText = require('./TtsText.js');
let TTSstate = require('./TTSstate.js');
let Input = require('./Input.js');
let I18nArgument = require('./I18nArgument.js');
let ASRSrvRequest = require('./ASRSrvRequest.js');
let I18nText = require('./I18nText.js');
let ASRStatus = require('./ASRStatus.js');
let actiontag = require('./actiontag.js');
let ASRSrvResponse = require('./ASRSrvResponse.js');
let AudioDeviceDescription = require('./AudioDeviceDescription.js');
let ASRLangModelMngmt = require('./ASRLangModelMngmt.js');
let TtsMark = require('./TtsMark.js');
let DirectionOfArrival = require('./DirectionOfArrival.js');
let ASRFileActionGoal = require('./ASRFileActionGoal.js');
let ASRFileActionResult = require('./ASRFileActionResult.js');
let TtsResult = require('./TtsResult.js');
let TtsActionGoal = require('./TtsActionGoal.js');
let AudioPlayAction = require('./AudioPlayAction.js');
let SoundGoal = require('./SoundGoal.js');
let TtsActionFeedback = require('./TtsActionFeedback.js');
let SoundActionFeedback = require('./SoundActionFeedback.js');
let TtsActionResult = require('./TtsActionResult.js');
let SoundResult = require('./SoundResult.js');
let TtsAction = require('./TtsAction.js');
let SoundFeedback = require('./SoundFeedback.js');
let AudioPlayGoal = require('./AudioPlayGoal.js');
let SoundAction = require('./SoundAction.js');
let AudioPlayActionFeedback = require('./AudioPlayActionFeedback.js');
let AudioPlayActionGoal = require('./AudioPlayActionGoal.js');
let ASRFileActionFeedback = require('./ASRFileActionFeedback.js');
let TtsGoal = require('./TtsGoal.js');
let SoundActionResult = require('./SoundActionResult.js');
let ASRFileResult = require('./ASRFileResult.js');
let TtsFeedback = require('./TtsFeedback.js');
let SoundActionGoal = require('./SoundActionGoal.js');
let ASRFileGoal = require('./ASRFileGoal.js');
let AudioPlayResult = require('./AudioPlayResult.js');
let AudioPlayActionResult = require('./AudioPlayActionResult.js');
let AudioPlayFeedback = require('./AudioPlayFeedback.js');
let ASRFileFeedback = require('./ASRFileFeedback.js');
let ASRFileAction = require('./ASRFileAction.js');

module.exports = {
  audiosignal: audiosignal,
  asrresult: asrresult,
  WebGuiEvent: WebGuiEvent,
  AudioPlayerState: AudioPlayerState,
  EnablingSoundLocalisation: EnablingSoundLocalisation,
  VoiceActivity: VoiceActivity,
  asrupdate: asrupdate,
  ASRLanguage: ASRLanguage,
  InputArgument: InputArgument,
  ASREvent: ASREvent,
  ASRActivation: ASRActivation,
  TtsText: TtsText,
  TTSstate: TTSstate,
  Input: Input,
  I18nArgument: I18nArgument,
  ASRSrvRequest: ASRSrvRequest,
  I18nText: I18nText,
  ASRStatus: ASRStatus,
  actiontag: actiontag,
  ASRSrvResponse: ASRSrvResponse,
  AudioDeviceDescription: AudioDeviceDescription,
  ASRLangModelMngmt: ASRLangModelMngmt,
  TtsMark: TtsMark,
  DirectionOfArrival: DirectionOfArrival,
  ASRFileActionGoal: ASRFileActionGoal,
  ASRFileActionResult: ASRFileActionResult,
  TtsResult: TtsResult,
  TtsActionGoal: TtsActionGoal,
  AudioPlayAction: AudioPlayAction,
  SoundGoal: SoundGoal,
  TtsActionFeedback: TtsActionFeedback,
  SoundActionFeedback: SoundActionFeedback,
  TtsActionResult: TtsActionResult,
  SoundResult: SoundResult,
  TtsAction: TtsAction,
  SoundFeedback: SoundFeedback,
  AudioPlayGoal: AudioPlayGoal,
  SoundAction: SoundAction,
  AudioPlayActionFeedback: AudioPlayActionFeedback,
  AudioPlayActionGoal: AudioPlayActionGoal,
  ASRFileActionFeedback: ASRFileActionFeedback,
  TtsGoal: TtsGoal,
  SoundActionResult: SoundActionResult,
  ASRFileResult: ASRFileResult,
  TtsFeedback: TtsFeedback,
  SoundActionGoal: SoundActionGoal,
  ASRFileGoal: ASRFileGoal,
  AudioPlayResult: AudioPlayResult,
  AudioPlayActionResult: AudioPlayActionResult,
  AudioPlayFeedback: AudioPlayFeedback,
  ASRFileFeedback: ASRFileFeedback,
  ASRFileAction: ASRFileAction,
};
