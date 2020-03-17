close all;
clear all;
clc;

%% Gravity term
worldGravity = iDynTree.Vector3();
worldGravity.zero();
worldGravity.setVal(2, -9.81);

%% Model parameters
modelPrefix = 'humanSubject';
modelSuffix = '_66dof.urdf';
modelNumber = "02";

%% Get model path
cd('../')
pwd;
modelDirPath = fullfile(pwd, modelPrefix + modelNumber);

addpath(genpath('../' + modelDirPath));
modelPath = fullfile(modelDirPath, "/" + modelPrefix + modelNumber + modelSuffix);


%% Load model to idyntree
modelLoader = iDynTree.ModelLoader();
if ~modelLoader.loadModelFromFile(char(modelPath))
    fprintf('Something wrong with the model loading.')
end

model = modelLoader.model();

%% Initialize base transform and velocity
baseTransform = iDynTree.Transform.Identity();
baseVelocity = iDynTree.Twist.Zero();

%% Initialize robot state to zero
jointPos = iDynTree.VectorDynSize();
jointPos.resize(model.getNrOfDOFs)
jointPos.zero();

jointVel = iDynTree.VectorDynSize;
jointVel.resize(model.getNrOfDOFs);
jointVel.zero();

%% Initialize kindyn and set robot state
kindynComp = iDynTree.KinDynComputations();
kindynComp.loadRobotModel(model);
kindynComp.setFloatingBase('Pelvis');

%% NOTE: SetRobotState with base arguments is not available through bindings ?
kindynComp.setRobotState(jointPos,...
                         jointVel,...
                         worldGravity);
                     
%% Get center of mass quantities
comPosition     = kindynComp.getCenterOfMassPosition();
comVelocity     = kindynComp.getCenterOfMassVelocity();
comAcceleration = kindynComp.getCenterOfMassBiasAcc();
