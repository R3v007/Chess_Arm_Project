classdef robotarm 

    properties
        device
        lobot
        magnetstate
        start
        arm
        chomp
        Joint_vals
        temp
        home
        final
        finalstruct
        optimpickconfig
        timestamppick
        solpickinfo
        homex
        homey
        homez
        servo_ids
    end

    methods
    
    function this = robotarm()
        % constructor should import the urdf, setup the serial comms, and
        % set the robot to home position
        % import library xarm=py.importlib.import_module('xarm')
        pyrun("import xarm");
        this.home=[-90.0, 0.0, 0.0, 0.0];
        this.servo_ids=[2, 3, 4, 5];
        this.arm=pyrun("arm = xarm.Controller('USB')","arm");
        this.lobot = importrobot("robot_arm_urdf/urdf/robot_arm_urdf.urdf", DataFormat="row");
        this.chomp = manipulatorCHOMP(this.lobot);
        disp("Robot Initialised");
        disp("Manipulator Chomp Initialised");

        % find_angles=input("Do you want to find the current joint angles? 1/0 \n");
        % if(find_angles)
        %     joints=this.getcurrentjointangles();
        %     disp("Current Joint angles are: ");
        %     disp(joints);
        %     wait("Press Enter to Continue");
        % end

        disp("Setting Robot to Home Position");
        % pyrun("arm.setPosition([[2,-90.0],[3,0.0],[4,0.0],[5,0.0]],wait=1)");
        isDone=this.setposition(this.home);
        disp(isDone);
        
        %show(this.lobot);    
        %this.device=serialport("COM9",9600);
        %writeline(this.device,string(0));   
        
    end
    
    function magnet(this,state)
        this.magnetstate=state;
        writeline(this.device,string(state));
        disp(["magnetstate sent to the UNo: ", this.magnetstate]);
    end

    function joints=getcurrentjointangles(this)      %get angle of each joint in radius
        joints(1)=pyrun("a=arm.getPosition(2,1)","a")*pi/180;
        joints(2)=pyrun("a=arm.getPosition(3,1)","a")*pi/180;
        joints(3)=pyrun("a=arm.getPosition(4,1)","a")*pi/180;
        joints(4)=pyrun("a=arm.getPosition(5,1)","a")*pi/180;
    end

    function status=set_to_home(this)
        rad_home=rad2deg(this.home(:));
        status=this.setposition(this.home);
    end
        

    function isDone=setposition(this,joints)
         pyrun("arm.setPosition([[2,a],[3,b],[4,c],[5,d]],wait=1)", ...
             a=joints(1),b=joints(2),c=joints(3),d=joints(4));
         isDone=true;
    end
    
    function joint_states=get_joint_states(this,coords) %get final angle of each joint in radians by ik
        temppose=trvec2tform(coords);
        ik = inverseKinematics( "RigidBodyTree",this.lobot);
        weights = [0 0 0 1 1 1];
        initialguess = homeConfiguration(this.lobot);
        [configSoln,~] = ik("L4",temppose,weights,initialguess);
        %show(this.lobot,this.finalstruct);

        % UNCOMMENT THIS IF NOT ON REV'S LAPTOP
        % joint_states=cell2mat({configSoln(1:4).JointPosition});

        % IF ON REV'S LAPTOP
        joint_states=configSoln;
        next_Step=input("Do you want to Visualise the coordinates?1/0 \n");
        switch next_Step
            case 1
                status=this.visualise_coords(coords);
                disp(status)
        end
    end
    
    function [waypoints, timestamps]=dochomp(this,coords, time, timestep)
        joints=this.getcurrentjointangles();
        final_state=this.get_joint_states(coords);
        disp(final_state)
        [waypoints,timestamps,this.solpickinfo] = optimize(this.chomp,[joints ; final_state ], ... % Starting and ending robot joint configurations
                                                                 [0 time], ...                    % Two waypoint times, first at 0s and last at 2s
                                                                 timestep, ...                     % 0.1s time step
                                                                 InitialTrajectoryFitType="minjerkpolytraj");
        %show(chomp,optimpickconfig);
        joints=rad2deg(final_state);
        % joints=joints.*-1;
        next_Step=input("Do you want to Visualise(0) or Run(1)? 1/0 \n");
        switch next_Step
            case 1
                status=this.setposition(joints);
                disp(status);
            case 0
                status=this.visualise_motion(waypoints);
                disp(status)
        end
   end

        % function to visualise the robot and the coordinates chosen. 
    function status=visualise_coords(this, coords)
        close all;
        show(this.lobot);
        hold on;
        plotTransforms(coords,eul2quat([0 0 0]),FrameSize=0.2);
        status=true;
    end

    function status=visualise_motion(this, waypoints)
        close all;
        show(this.lobot);
        hold on;
        show(this.chomp, waypoints);
        title("Optimized Picking Trajectory")
        xlim([-1 1])
        ylim([-1 1])
        zlim([-0.2 1])
        status=true;
    end
    
    %find homeposition x y z and joint angles
    function changeonechessposition(this,pickx,picky,pickz,finalx,finaly,finalz)
        %hometopick,magneton,picktohome,hometofinal,magnetoff,finaltohome
        this.dochomp(pickx,picky,pickz);
        this.magnet(1);
        this.dochomp(0,0,0); %change this later
        this.dochomp(finalx,finaly,finalz);
        this.magnet(0);
        this.dochomp(0,0,0);%change this later
    end

    end

end
