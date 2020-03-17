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

%% Test Link Symmetry
linkPrefixes = ["Left", "Right"];
linkNames = ["UpperLeg",...
             "LowerLeg",...
             "Foot",...
             "Toe",...
             "Shoulder",...
             "UpperArm",...
             "ForeArm",...
             "Hand"];

linkPosTolerance = 1e-5;
         
%% Iterate over links
for i = 1:size(linkNames, 2)
    
    leftLinkName  = linkPrefixes(1) + linkNames(i);
    rightLinkName = linkPrefixes(2) + linkNames(i);
    
    fprintf("============================================================== \n");
    fprintf("Comparing %s and %s links transforms \n", leftLinkName, rightLinkName);
    
    % Get link transforms
    leftLinkTransform  = kindynComp.getWorldTransform(model.getLinkIndex(char(leftLinkName)));
    rightLinkTransform = kindynComp.getWorldTransform(model.getLinkIndex(char(rightLinkName)));
    
    for p = 1:3
        
        % Check if the transform positions are equal in magnitude
        if (abs(leftLinkTransform.getPosition().getVal(p-1)) - ...
            abs(rightLinkTransform.getPosition().getVal(p-1)) > linkPosTolerance)
        
            fprintf('Position tolerance for index %i is not matching \n', p-1)
            fprintf('%s position at index %i is %d \n', leftLinkName, p-1, leftLinkTransform.getPosition().getVal(p-1));
            fprintf('%s position at index %i is %d \n', rightLinkName, p-1, rightLinkTransform.getPosition().getVal(p-1));
            
        else
            fprintf('Link positions at index %i are matching \n', p-1)
        end
            
    end
    
end
