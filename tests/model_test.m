close all;
clear all;
clc;
warning off backtrace

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

% Get intertial parameters
modelInertialParameters = iDynTree.VectorDynSize();

if (~model.getInertialParameters(modelInertialParameters))
    error("Failed to get model inertial parameters");
end

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
    
    % Get link name
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
        
            warning('Position tolerance for index %i is not matching \n', p-1)
            warning('%s position at index %i is %f \n', leftLinkName, p-1, leftLinkTransform.getPosition().getVal(p-1));
            warning('%s position at index %i is %f \n', rightLinkName, p-1, rightLinkTransform.getPosition().getVal(p-1));
            
        else
            fprintf('Link positions at index %i are matching \n', p-1)
        end
            
    end
    
    fprintf("----------------------------- \n");
    fprintf("Comparing inertial parameters \n");
    
    % Get link index
    leftLinkIndex  = model.getLinkIndex(char(leftLinkName));
    rightLinkIndex = model.getLinkIndex(char(rightLinkName));
    
    % Get link mass
    leftLinkMass  = modelInertialParameters.getVal(10 * leftLinkIndex + 0);
    rightLinkMass = modelInertialParameters.getVal(10 *rightLinkIndex + 0);
    
    % Check link mass
    if (leftLinkMass ~= rightLinkMass)
        warning('Mismatch in mass');
        warning('%s link mass is %f Kgs', leftLinkName, leftLinkMass);
        warning('%s link mass is %f Kgs', rightLinkName, rightLinkMass);
    else
        fprintf('Link mass values matching\n');
    end
    
    % Get link center of mass
    leftLinkCOM  = [modelInertialParameters.getVal(10 * leftLinkIndex + 1),...
                    modelInertialParameters.getVal(10 * leftLinkIndex + 2),...
                    modelInertialParameters.getVal(10 * leftLinkIndex + 3)];
               
    rightLinkCOM = [modelInertialParameters.getVal(10 * rightLinkIndex + 1),...
                    modelInertialParameters.getVal(10 * rightLinkIndex + 2),...
                    modelInertialParameters.getVal(10 * rightLinkIndex + 3)];
                
    % Check link center of mass
    comMismatch = false;
    for c = 1:3
        if (abs(leftLinkCOM(c)) ~= abs(rightLinkCOM(c)))
            comMismatch = true;
            warning('Mismatch in center of mass position')
            break
        end
    end
    
    if (comMismatch)
        warning('%s link center of mass is (%f, %f, %f)', leftLinkName,...
                                                          leftLinkCOM(1),...
                                                          leftLinkCOM(2),...
                                                          leftLinkCOM(3));
        warning('%s link center of mass is (%f, %f, %f)', rightLinkName,...
                                                          rightLinkCOM(1),...
                                                          rightLinkCOM(2),...
                                                          rightLinkCOM(3));
    else
        fprintf('Link center of mass position values matching\n');
    end
    
    % Get link inertial elements
    leftLinkInertia  = [modelInertialParameters.getVal(10 * leftLinkIndex + 4),...
                        modelInertialParameters.getVal(10 * leftLinkIndex + 5),...
                        modelInertialParameters.getVal(10 * leftLinkIndex + 6),...
                        modelInertialParameters.getVal(10 * leftLinkIndex + 7),...
                        modelInertialParameters.getVal(10 * leftLinkIndex + 8),...
                        modelInertialParameters.getVal(10 * leftLinkIndex + 9)];
                   
    rightLinkInertia = [modelInertialParameters.getVal(10 * rightLinkIndex + 4),...
                        modelInertialParameters.getVal(10 * rightLinkIndex + 5),...
                        modelInertialParameters.getVal(10 * rightLinkIndex + 6),...
                        modelInertialParameters.getVal(10 * rightLinkIndex + 7),...
                        modelInertialParameters.getVal(10 * rightLinkIndex + 8),...
                        modelInertialParameters.getVal(10 * rightLinkIndex + 9)];
    
    % Check link inertia
    inertiaMismatch = false;
    for j = 1:6
        if (abs(leftLinkInertia(j)) ~= abs(rightLinkInertia(j)))
            inertiaMismatch = true;
            warning('Mismatch in inertia')
            break
        end
    end
    
    if (inertiaMismatch)
        warning('%s link inertia elements are (%f, %f, %f, %f, %f, %f)',...
                                                          leftLinkName,...
                                                          leftLinkInertia(1),...
                                                          leftLinkInertia(2),...
                                                          leftLinkInertia(3),...
                                                          leftLinkInertia(4),...
                                                          leftLinkInertia(5),...
                                                          leftLinkInertia(6));
        warning('%s link inertia elements are (%f, %f, %f, %f, %f, %f)',...
                                                          rightLinkName,...
                                                          rightLinkInertia(1),...
                                                          rightLinkInertia(2),...
                                                          rightLinkInertia(3),...
                                                          rightLinkInertia(4),...
                                                          rightLinkInertia(5),...
                                                          rightLinkInertia(6));
    else
        fprintf('Link inertia values matching\n');
    end
                
        
end
