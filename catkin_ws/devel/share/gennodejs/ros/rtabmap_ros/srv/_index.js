
"use strict";

let GetPlan = require('./GetPlan.js')
let ResetPose = require('./ResetPose.js')
let ListLabels = require('./ListLabels.js')
let GetMap = require('./GetMap.js')
let SetLabel = require('./SetLabel.js')
let GetNodeData = require('./GetNodeData.js')
let GetMap2 = require('./GetMap2.js')
let AddLink = require('./AddLink.js')
let GetNodesInRadius = require('./GetNodesInRadius.js')
let PublishMap = require('./PublishMap.js')
let SetGoal = require('./SetGoal.js')

module.exports = {
  GetPlan: GetPlan,
  ResetPose: ResetPose,
  ListLabels: ListLabels,
  GetMap: GetMap,
  SetLabel: SetLabel,
  GetNodeData: GetNodeData,
  GetMap2: GetMap2,
  AddLink: AddLink,
  GetNodesInRadius: GetNodesInRadius,
  PublishMap: PublishMap,
  SetGoal: SetGoal,
};
