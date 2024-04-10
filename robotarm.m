classdef robotarm

    properties
        device
        lobot
        magnetstate
    end

    methods
    
    function this = robotarm()
        % constructor should import the urdf, setup the serial comms, and
        % set the robot to home position
    this.lobot = importrobot("robot_arm_urdf\urdf\robot_arm_urdf.urdf", DataFormat="row");
    show(this.lobot);    
    this.device=serialport("COM9",9600);
    writeline(this.device,string(0));    
    end
    
    function magnet(this,state)
    this.magnetstate=state;
    writeline(this.device,string(state));
    disp(["magnetstate sent to the UNo: ", this.magnetstate]);
    end

    end

end
