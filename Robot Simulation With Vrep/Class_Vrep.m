classdef Class_Vrep < handle
    
    properties
        Joint
        Base
        Cubid
        Suction
        vrep
        clientID
    end
    
    methods
        function this = Class_Vrep()
            this.Joint   = struct(   'res'       ,   zeros(1,6),...
                                    'handle'    ,   zeros(1,6),...
                                    'pos'       ,   zeros(1,6));
            this.Base    = struct(   'res'       ,   [],...
                                    'handle'    ,   [],...
                                    'pos'       ,   [],...
                                    'link0_handle', []);
            this.Cubid   = struct(   'res'       ,   [],...
                                    'handle'    ,   [],...
                                    'pos'       ,   [],...
                                    'height'    ,   14.5,...
                                    'distance'  ,   10);
            this.Suction = struct(   'res'       ,   [],...
                                    'handle'    ,   [],...
                                    'able'      ,   []);
            this.vrep        = [];
            this.clientID    = [];
        end
        
        function Initial_vrep(this)
            % >>>> Start Simulation
            this.vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
            this.vrep.simxFinish(-1);        % just in case, close all opened connections
            this.clientID = this.vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
            this.vrep.simxStartSimulation(this.clientID, this.vrep.simx_opmode_blocking);
        end
        
        function Get_handle(this)
            % >>>> Get robot base handle
            [this.Base.res, this.Base.handle] = this.vrep.simxGetObjectHandle(     this.clientID,   'my_robot_base',    this.vrep.simx_opmode_blocking);
            [this.Base.res, this.Base.pos]    = this.vrep.simxGetObjectPosition(   this.clientID,   this.Base.handle,    this.Base.handle,  this.vrep.simx_opmode_blocking);
            [this.Base.res, this.Base.link0_handle] = this.vrep.simxGetObjectHandle(     this.clientID,   'link0_vis',    this.vrep.simx_opmode_blocking);
            
            % >>>> Get robot joint handle
            for i = 1:6
                [this.Joint.res(i), this.Joint.handle(i)] = this.vrep.simxGetObjectHandle( this.clientID,   ['joint' num2str(i)],   this.vrep.simx_opmode_blocking);
                [this.Joint.res(i), this.Joint.pos(i)]    = this.vrep.simxGetJointPosition(this.clientID,   this.Joint.handle(i),    this.vrep.simx_opmode_streaming);
            end
            
            % >>>> Get Cubid handle
            [this.Cubid.res, this.Cubid.handle] = this.vrep.simxGetObjectHandle(   this.clientID,   'Cuboid',           this.vrep.simx_opmode_blocking);
            [this.Cubid.res, this.Cubid.pos]    = this.vrep.simxGetObjectPosition( this.clientID,   this.Cubid.handle,   this.Base.link0_handle,   this.vrep.simx_opmode_blocking);
            
            % >>>> Get Suction handle
            [this.Suction.res, this.Suction.handle] = this.vrep.simxGetObjectHandle(     this.clientID,   'suctionPad',    this.vrep.simx_opmode_blocking);
            
        end
        
        function Start_sync(this)
            this.vrep.simxSynchronous(this.clientID, true);
        end
        
        function Sent_JointPosCmd_Quick(this, JointCmd)
            for j = 1:6
                this.vrep.simxSetJointTargetPosition(this.clientID, this.Joint.handle(j), JointCmd(j), this.vrep.simx_opmode_oneshot );
            end
        end
        
        function Sent_JointPosCmd(this, JointCmd)
            this.vrep.simxPauseCommunication(this.clientID, true);
            for j = 1:6
                this.vrep.simxSetJointTargetPosition(this.clientID, this.Joint.handle(j), JointCmd(j), this.vrep.simx_opmode_oneshot );
            end
            this.vrep.simxPauseCommunication(this.clientID, false);
        end
              
        function Get_JointPos(this)
            for j = 1:6
                [this.Joint.res(j), this.Joint.pos(j)]    = this.vrep.simxGetJointPosition(this.clientID,  this.Joint.handle(j),        this.vrep.simx_opmode_streaming);
            end
        end
        
        function Get_CubidPos(this)
            [this.Cubid.res, this.Cubid.pos]    = this.vrep.simxGetObjectPosition( this.clientID,   this.Cubid.handle,   this.Base.link0_handle,   this.vrep.simx_opmode_blocking);
        end
        
        function z = Get_Object(this)

            [err, maxval] = this.vrep.simxGetObjectFloatParameter(this.clientID, this.Cubid.handle, this.vrep.sim_thisfloatparam_modelbbox_max_z, this.vrep.simx_opmode_blocking);
            
            z = maxval;
        end
        
        function Trigger(this)
            this.vrep.simxSynchronousTrigger(this.clientID);
        end
        
        function Set_Suction(this, type)
            switch type
                case 'able'
                    this.vrep.simxSetIntegerSignal(this.clientID, this.suction, 1, this.vrep.simx_opmode_oneshot);
                    [~, this.Suction.able] = this.vrep.simxGetIntegerSignal(this.clientID, this.suction, this.vrep.simx_opmode_blocking);
                case 'enable'
                    this.vrep.simxSetIntegerSignal(this.clientID, this.suction, 0, this.vrep.simx_opmode_oneshot);
                    [~, this.Suction.able] = this.vrep.simxGetIntegerSignal(this.clientID, this.suction, this.vrep.simx_opmode_blocking);
                    
            end
        end
        
        function StopSimulation(this)
            this.vrep.simxStopSimulation(this.clientID, this.vrep.simx_opmode_oneshot_wait);
            this.vrep.simxFinish(this.clientID);
            this.vrep.delete();
        end
    end
    
end