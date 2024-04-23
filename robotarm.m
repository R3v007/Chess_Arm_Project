classdef robotarm 

    properties
        device
        lobot
        magnetstate
        start
        arm
        chomp
        Joint_vals
        home
        servo_ids
        dh_params
    end

    methods
    
        function [this, record_joints] = robotarm()
        % constructor should import the urdf, setup the serial comms, and
        % set the robot to home position
        % import library xarm=py.importlib.import_module('xarm')
        % pyrun("import xarm");
        this.servo_ids=[2, 3, 4, 5];
        % this.arm=pyrun("arm = xarm.Controller('USB')","arm");
        this.lobot = importrobot("robot_arm_urdf/urdf/robot_arm_urdf.urdf", DataFormat="row");
        this.chomp = manipulatorCHOMP(this.lobot);
        disp("Robot Initialised");
        disp("Manipulator Chomp Initialised");

        % find_angles=input("Do you want to find the current joint angles? 1/0 \n");
        % if(find_angles)
        %     record_joints=this.getcurrentjointangles();
        %     disp("Current Joint angles are: ");
        %     disp(record_joints);
        %     pause(2);
        % end

        % SETTING THE DH PARAMS a alpha d theta
        this.dh_params=[ -0.015 -pi/2 0.035 0;
                         0.097  pi     0    0;
                         0.097  pi     0    0;
                         0      pi/2   0    0];
        disp(this.dh_params);
        

        disp("Setting Robot to Home Position");
        % pyrun("arm.setPosition([[2,-30.0],[3,0.0],[4,0.0],[5,0.0]],wait=1)");
        % this.home=this.getcurrentjointangles(); 

        this.home=[-90 0 0 0];
        %this.device=serialport("COM9",9600);
        %writeline(this.device,string(0));   
        
    end
    
    function magnet(this,state)
        this.magnetstate=state;
        writeline(this.device,string(state));
        disp(["magnetstate sent to the Uno: ", this.magnetstate]);
    end

    function joints=getcurrentjointangles(this)      %get angle of each joint in radius
        joints(1)=pyrun("a=arm.getPosition(2,1)","a")*pi/180;
        joints(2)=pyrun("a=arm.getPosition(3,1)","a")*pi/180;
        joints(3)=pyrun("a=arm.getPosition(4,1)","a")*pi/180;
        joints(4)=pyrun("a=arm.getPosition(5,1)","a")*pi/180;
    end

    function status=set_to_home(this)
        % rad_home=rad2deg(this.home(:));
        status=this.setposition(this.home);
    end
        

    function isDone=setposition(this,joints)
         % pyrun("arm.setPosition([[2,a],[3,b],[4,c],[5,d]],wait=1)", ...
             % a=joints(1),b=joints(2),c=joints(3),d=joints(4));
         isDone=true;
    end
    
    function joint_states=get_joint_states(this,coords) %get final angle of each joint in radians by ik
        temppose=trvec2tform(coords);
        ik = inverseKinematics( "RigidBodyTree",this.lobot);
        weights = [1 1 1 1 1 1];
        initialguess = homeConfiguration(this.lobot);
        [configSoln,~] = ik("L4",temppose,weights,initialguess);
        %show(this.lobot,this.finalstruct);

        % UNCOMMENT THIS IF NOT ON REV'S LAPTOP
        % joint_states=cell2mat({configSoln(1:4).JointPosition});

        % IF ON REV'S LAPTOP
        joint_states=configSoln;
        this.visualise_coords(coords);
    end

    function isOver=update_home(this)
        % joints_rn=this.getcurrentjointangles();
        disp("This is the current joints");
        disp(joints_rn)
        yn=input("Do you want to update? 1(yes)/0(no)");
        if(yn)
            this.home=joints_rn;
            disp("Home Updated");
            disp(this.home);
            isOver=true;
        end
    end    
    
    function [final_state,waypoints, timestamps]=dochomp(this,coords, time, timestep)
        % curr_joints=this.getcurrentjointangles();
        disp("Starting joint angles: ideally this should be home");
        % disp(curr_joints)
        disp("this is home angles:")
        disp(this.home);
        

        final_state=this.get_joint_states(coords);
        disp("Final joint angles")
        disp(final_state)
        [waypoints,timestamps,~] = optimize(this.chomp,[this.home ; final_state ], ... % Starting and ending robot joint configurations
                                                                 [0 time], ...                    % Two waypoint times, first at 0s and last at 2s
                                                                 timestep, ...                     % 0.1s time step
                                                                 InitialTrajectoryFitType="quinticpolytraj");
        n=length(timestamps);
        disp("the final angles that the chomp outputs:");
        disp(waypoints(n,:));

   end

        % function to visualise the robot and the coordinates chosen. 
    function status=visualise_coords(this, coords)
        this.show_current_arm()
        hold on;
        plotTransforms(coords,eul2quat([0 0 0]),FrameSize=0.2);
        status=true;
    end

    function status=visualise_motion(this, joints)
        % close all;
        this.show_current_arm()
        hold on;
        hold on 
        show(this.lobot, joints, PreservePlot=false);
        % show(this.chomp, waypoints);
        % title("Optimized Picking Trajectory")
        % xlim([-1 1])
        % ylim([-1 1])
        % zlim([-0.2 1])
        status=true;
    end

    function status=visualise_motion_chomp(this, waypoints)
        % close all;
        this.show_current_arm();
        hold on;
        show(this.chomp, waypoints);
        title("Optimized Picking Trajectory")
        status=true;
    end

    function status=show_current_arm(this)
        % joints=this.getcurrentjointangles;
        joints=this.home;
        show(this.lobot);
        hold on;
        show(this.lobot, joints, PreservePlot=false);
        status=true;
    end
    
    end

end
