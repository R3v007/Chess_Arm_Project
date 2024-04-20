classdef robotarm   <handle

    properties
        device
        lobot
        magnetstate
        start
        arm
        J1
        J2
        J3
        J4
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
    end

    methods
    
    function this = robotarm()
        % constructor should import the urdf, setup the serial comms, and
        % set the robot to home position
    import library xarm=py.importlib.import_module('xarm')
    pyrun("import xarm");
    this.arm=pyrun("arm = xarm.Controller('USB')","arm");
    pyrun("arm.setPosition([[2,-90.0],[3,0.0],[4,0.0],[5,0.0]],wait=1)");
    this.lobot = importrobot("robot_arm_urdf.urdf", DataFormat="row");
    %show(this.lobot);    
    %this.device=serialport("COM9",9600);
    %writeline(this.device,string(0));    
    end
    
    function magnet(this,state)
    this.magnetstate=state;
    writeline(this.device,string(state));
    disp(["magnetstate sent to the UNo: ", this.magnetstate]);
    end

    %start joints configuration: getangle from pythonlib
    %end joints configuration: desired L4 coordinate, ik to get joints config
    %perform chomp with start & end joints configuration
    %setangle from pythonlib
    %end
    %setPosition(2,startwith-120)
    %setPosition(3,startwith0)
    %setPosition(4,startwith0)
    %setPosition(5,startwith0)

    function getcurrentjointangle(this)      %get angle of each joint in radius
    this.J1=pyrun("a=arm.getPosition(2,1)","a")*pi/180;
    this.J2=pyrun("a=arm.getPosition(3,1)","a")*pi/180;
    this.J3=pyrun("a=arm.getPosition(4,1)","a")*pi/180;
    this.J4=pyrun("a=arm.getPosition(5,1)","a")*pi/180;
    this.home=[this.J1,this.J2,this.J3,this.J4];
    end

    function setposition(this,J1new,J2new,J3new,J4new)
    pyrun("arm.setPosition([[2,a],[3,b],[4,c],[5,d]],wait=1)",a=J1new,b=J2new,c=J3new,d=J4new);
    end
    
    function getfinaljointangle(this,x,y,z) %get final angle of each joint in radius by ik
    this.temp=[x,y,z];
    temppose=trvec2tform(this.temp);
    ik = inverseKinematics( "RigidBodyTree",this.lobot);
    weights = [0 0 0 1 1 1];
    initialguess = homeConfiguration(this.lobot);
    [this.finalstruct,solnInfo] = ik("L4",temppose,weights,initialguess);
    %show(this.lobot,this.finalstruct);
    this.final=cell2mat({configSoln(1:4).JointPosition});
    end
    
    function dochomp(this,x,y,z)
    this.getcurrentjointangle;
    this.getfinaljointangle(x,y,z);
    chomp = manipulatorCHOMP(this.lobot);
    [this.optimpickconfig,this.timestamppick,this.solpickinfo] = optimize(chomp,[this.home ; this.final ], ... % Starting and ending robot joint configurations
                                                             [0 2], ...                    % Two waypoint times, first at 0s and last at 2s
                                                             0.10, ...                     % 0.1s time step
                                                             InitialTrajectoryFitType="minjerkpolytraj");
    %show(chomp,optimpickconfig);
    joints=rad2deg(this.optimpickconfig(21,:)); 
    this.setposition(joints(1),joints(2),joints(3),joints(4));
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
