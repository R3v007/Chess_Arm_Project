classdef robotarm_joints

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
        tf_matrix
        locations
        states
    end

    methods
    
        function [this, record_joints] = robotarm_joints()

        pyrun("import xarm");
        this.servo_ids=[2, 5, 3, 4];
        this.home=[0 90 45 0 0];
        joints=deg2rad(this.home);
        this.arm=pyrun("arm = xarm.Controller('USB')","arm");
        this.lobot = importrobot("robot_arm_urdf/urdf/robot_arm_urdf.urdf", DataFormat="row");
        this.chomp = manipulatorCHOMP(this.lobot);
        disp("Robot Initialised");
        disp("Manipulator Chomp Initialised");

            
        find_angles=input("Do you want to find the current joint angles? 1/0 \n");
        if(find_angles)
            [~,record_joints]=this.getcurrentjointangles();
            disp("Current Joint angles are: ");
            disp(record_joints);
            pause(2);
            
        end
               
        this.locations=readtable("data_points.xls");
        this.states(:,:,1)=[this.locations.Var1(:) this.locations.Var2(:) ...
                        this.locations.Var3(:) this.locations.Var4(:)];
        this.states(:,:,2)=[this.locations.Var5(:) this.locations.Var6(:) ...
                        this.locations.Var7(:) this.locations.Var8(:)];
        this.states(:,:,3)=[this.locations.Var9(:) this.locations.Var10(:) ...
                        this.locations.Var11(:) this.locations.Var12(:)];
        

        disp("Setting Robot to Home Position");
        this.set_to_home();


        disp("Serial Comms Established for the Electromagnet")
        % this.device=serialport("COM9",9600);
        % writeline(this.device,string(0));   
        disp("Ready to Begin Demo");
        
    end
    
    function magnet(this,state)
        this.magnetstate=state;
        % writeline(this.device,string(state));
        disp(["magnetstate sent to the Uno: ", this.magnetstate]);
    end

    function [og_joints, mod_joints]=getcurrentjointangles(this)      %get angle of each joint in radius
        this_joints(1)=pyrun("a=arm.getPosition(2,1)","a")*pi/180;
        this_joints(2)=pyrun("a=arm.getPosition(5,1)","a")*pi/180;
        this_joints(3)=pyrun("a=arm.getPosition(3,1)","a")*pi/180;
        this_joints(4)=pyrun("a=arm.getPosition(4,1)","a")*pi/180;
        og_joints=this_joints;
        og_joints(5)=0;

        % modifications to be made to the joint angles 
        % make the simulation match reality
        mod_joints=this_joints;
        mod_joints(2)=this_joints(2)-pi/2;
        mod_joints(3)=this_joints(3)-pi/4;
        mod_joints(4)=this_joints(4)*-1;
        mod_joints(5)=0;
    end

    function status=set_to_home(this)
        disp(this.home);
        status=this.setposition(this.home(1:4));
    end
        

    function isDone=setposition(this,joints)
         pyrun("arm.setPosition([[2,a],[5,b],[3,c],[4,d]],wait=1)", ...
             a=joints(1),b=joints(2),c=joints(3),d=joints(4));
         isDone=true;
    end
    
    function joint_states=get_joint_states(this,coords) %get final angle of each joint in radians by ik
        temppose=trvec2tform(coords);
        ik = inverseKinematics( "RigidBodyTree",this.lobot);
        weights = [1 1 1 1 1 1];
        initialguess = homeConfiguration(this.lobot);
        [configSoln,~] = ik("EE",temppose,weights,initialguess);
        %show(this.lobot,this.finalstruct);

        % UNCOMMENT THIS IF NOT ON REV'S LAPTOP
        % joint_states=cell2mat({configSoln(1:4).JointPosition});

        % IF ON REV'S LAPTOP
        joint_states=configSoln;
        this.visualise_coords(coords);
    end

    function isOver=update_home(this)
        joints_rn=this.getcurrentjointangles();
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
        [~,joints]=this.getcurrentjointangles;
        show(this.lobot);
        hold on;
        show(this.lobot, joints, PreservePlot=false);
        status=true;
    end

    function [Tf_mat, coords]=get_curr_coords(this)
        joints=this.getcurrentjointangles();  % get current joints
        %update dh params
        this.dh_params(1,4)=joints(1);
        this.dh_params(2,4)=joints(2);
        this.dh_params(3,4)=joints(3);
        this.dh_params(4,4)=joints(4);
        %calculate tf
        for i=1:5
            vals=this.dh_params(i,:);
            a=vals(1); alpha=vals(2); d=vals(3); theta=vals(4);
            
            T_matrix(:,:,i)=[ cos(theta) (-1*sin(theta)*cos(alpha)) (sin(theta)*sin(alpha)) a*cos(theta);
                       sin(theta)  (cos(theta)*cos(alpha))  (-1*cos(theta)*sin(alpha)) a*sin(theta);
                       0 sin(alpha) cos(alpha) d;
                       0 0 0 1];
        end
        Tf_mat=mtimes(T_matrix(:,:,5),T_matrix(:,:,4)); ...
        Tf_mat=mtimes(Tf_mat, T_matrix(:,:,3));
        Tf_mat=mtimes(Tf_mat, T_matrix(:,:,2));
        Tf_mat=mtimes(Tf_mat, T_matrix(:,:,1));
        coords=Tf_mat(1:3, 4);
    end

    end
end




