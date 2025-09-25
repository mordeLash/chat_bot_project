
"use strict";

let Recognizer = require('./Recognizer.js')
let SelectTexturedObject = require('./SelectTexturedObject.js')
let ChangeObjectRecognizerModel = require('./ChangeObjectRecognizerModel.js')
let StartEnrollment = require('./StartEnrollment.js')
let SetDatabase = require('./SetDatabase.js')
let StopEnrollment = require('./StopEnrollment.js')
let AddTexturedObject = require('./AddTexturedObject.js')

module.exports = {
  Recognizer: Recognizer,
  SelectTexturedObject: SelectTexturedObject,
  ChangeObjectRecognizerModel: ChangeObjectRecognizerModel,
  StartEnrollment: StartEnrollment,
  SetDatabase: SetDatabase,
  StopEnrollment: StopEnrollment,
  AddTexturedObject: AddTexturedObject,
};
