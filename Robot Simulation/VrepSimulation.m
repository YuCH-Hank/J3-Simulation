classdef VrepSimulation < handle
    
    properties
        Joint
        Base
        Cubid
        Suction
        vrep
        clientID
    end
    
    methods
        function obj = VrepSimulation()
            obj.Joint   = struct(   'res'       ,   zeros(1,6),...
                                    'handle'    ,   zeros(1,6),...
                                    'pos'       ,   zeros(1,6));
            obj.Base    = struct(   'res'       ,   [],...
                                    'handle'    ,   [],...
                                    'pos'       ,   [],...
                                    'link0_handle', []);
            obj.Cubid   = struct(   'res'       ,   [],...
                                    'handle'    ,   [],...
                                    'pos'       ,   [],...
                                    'height'    ,   14.5,...
                                    'distance'  ,   10);
            obj.Suction = struct(   'res'       ,   [],...
                                    'handle'    ,   [],...
                                    'able'      ,   []);
            obj.vrep        = [];
            obj.clientID    = [];
        end
        
        function Initial_vrep(obj)
            % >>>> Start Simulation
            obj.vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
            obj.vrep.simxFinish(-1);        % just in case, close all opened connections
            obj.clientID = obj.vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
            obj.vrep.simxStartSimulation(obj.clientID, obj.vrep.simx_opmode_blocking);
        end
        
        function Get_handle(obj)
            % >>>> Get robot base handle
            [obj.Base.res, obj.Base.handle] = obj.vrep.simxGetObjectHandle(     obj.clientID,   'my_robot_base',    obj.vrep.simx_opmode_blocking);
            [obj.Base.res, obj.Base.pos]    = obj.vrep.simxGetObjectPosition(   obj.clientID,   obj.Base.handle,    obj.Base.handle,  obj.vrep.simx_opmode_blocking);
            [obj.Base.res, obj.Base.link0_handle] = obj.vrep.simxGetObjectHandle(     obj.clientID,   'link0_vis',    obj.vrep.simx_opmode_blocking);
            
            % >>>> Get robot joint handle
            for i = 1:6
                [obj.Joint.res(i), obj.Joint.handle(i)] = obj.vrep.simxGetObjectHandle( obj.clientID,   ['joint' num2str(i)],   obj.vrep.simx_opmode_blocking);
                [obj.Joint.res(i), obj.Joint.pos(i)]    = obj.vrep.simxGetJointPosition(obj.clientID,   obj.Joint.handle(i),    obj.vrep.simx_opmode_streaming);
            end
            
            % >>>> Get Cubid handle
            [obj.Cubid.res, obj.Cubid.handle] = obj.vrep.simxGetObjectHandle(   obj.clientID,   'Cuboid',           obj.vrep.simx_opmode_blocking);
            [obj.Cubid.res, obj.Cubid.pos]    = obj.vrep.simxGetObjectPosition( obj.clientID,   obj.Cubid.handle,   obj.Base.link0_handle,   obj.vrep.simx_opmode_blocking);
            
            % >>>> Get Suction handle
            [obj.Suction.res, obj.Suction.handle] = obj.vrep.simxGetObjectHandle(     obj.clientID,   'suctionPad',    obj.vrep.simx_opmode_blocking);
            
        end
        
        function Start_sync(obj)
            obj.vrep.simxSynchronous(obj.clientID, true);
        end
        
        function Sent_JointPosCmd_Quick(obj, JointCmd)
            for j = 1:6
                obj.vrep.simxSetJointTargetPosition(obj.clientID, obj.Joint.handle(j), JointCmd(j), obj.vrep.simx_opmode_oneshot );
            end
        end
        
        function Sent_JointPosCmd(obj, JointCmd)
            obj.vrep.simxPauseCommunication(obj.clientID, true);
            for j = 1:6
                obj.vrep.simxSetJointTargetPosition(obj.clientID, obj.Joint.handle(j), JointCmd(j), obj.vrep.simx_opmode_oneshot );
            end
            obj.vrep.simxPauseCommunication(obj.clientID, false);
        end
              
        function Get_JointPos(obj)
            for j = 1:6
                [obj.Joint.res(j), obj.Joint.pos(j)]    = obj.vrep.simxGetJointPosition(obj.clientID,  obj.Joint.handle(j),        obj.vrep.simx_opmode_streaming);
            end
        end
        
        function Get_CubidPos(obj)
            [obj.Cubid.res, obj.Cubid.pos]    = obj.vrep.simxGetObjectPosition( obj.clientID,   obj.Cubid.handle,   obj.Base.link0_handle,   obj.vrep.simx_opmode_blocking);
        end
        
        function z = Get_Object(obj)

            [err, maxval] = obj.vrep.simxGetObjectFloatParameter(obj.clientID, obj.Cubid.handle, obj.vrep.sim_objfloatparam_modelbbox_max_z, obj.vrep.simx_opmode_blocking);
            
            z = maxval;
        end
        
        function Trigger(obj)
            obj.vrep.simxSynchronousTrigger(obj.clientID);
        end
        
        function Set_Suction(obj, type)
            switch type
                case 'able'
                    obj.vrep.simxSetIntegerSignal(obj.clientID, obj.suction, 1, obj.vrep.simx_opmode_oneshot);
                    [~, obj.Suction.able] = obj.vrep.simxGetIntegerSignal(obj.clientID, obj.suction, obj.vrep.simx_opmode_blocking);
                case 'enable'
                    obj.vrep.simxSetIntegerSignal(obj.clientID, obj.suction, 0, obj.vrep.simx_opmode_oneshot);
                    [~, obj.Suction.able] = obj.vrep.simxGetIntegerSignal(obj.clientID, obj.suction, obj.vrep.simx_opmode_blocking);
                    
            end
        end
        
        function StopSimulation(obj)
            obj.vrep.simxStopSimulation(obj.clientID, obj.vrep.simx_opmode_oneshot_wait);
            obj.vrep.simxFinish(obj.clientID);
            obj.vrep.delete();
        end
    end
    
end