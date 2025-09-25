
"use strict";

let GetSpeechDuration = require('./GetSpeechDuration.js')
let ASRService = require('./ASRService.js')
let SoundLocalisationService = require('./SoundLocalisationService.js')
let recognizerService = require('./recognizerService.js')

module.exports = {
  GetSpeechDuration: GetSpeechDuration,
  ASRService: ASRService,
  SoundLocalisationService: SoundLocalisationService,
  recognizerService: recognizerService,
};
