arm=robotarm

% THE Z POSITION IS FLIPPED SO MAKE SURE TO KEEP THAT IN MIND WHILE DOING
% SHIT
close all;
arm.show_current_arm(); 
%%
close all;
pick_coords=[-0.1000 0 0.0100];
place_coords=[-0.1700 0 0.0100];
arm.visualise_coords(pick_coords);
hold on;
arm.visualise_coords(place_coords);
%%
[joints_pick,waypoints_pick,times_pick]=arm.dochomp(pick_coords, 3, 0.1);
arm.visualise_motion(joints_pick);
pause(5)
arm.visualise_motion_chomp(waypoints_pick);
pause(5);
arm.set_to_home();
[joints_place,waypoints_place,times_place]=arm.dochomp(place_coords,3, 0.1);
arm.visualise_motion(joints_place);
pause(5)
arm.visualise_motion_chomp(waypoints_place);
pause(5);
arm.set_to_home();
%%  coordinate finder
y_start=0; y_end=-0.1; 
x_end=-0.3; x_start=-0.1;
x_step=abs((x_start-x_end)/8);
y_step=abs((y_Start-y_end)/8);
for i=1:8
    for j=1:8

        test_coords=[(x_start-(i-1)*x_step) (y_Start+(j-1)*y_step) 0.01];
        disp(test_coords)
        arm.visualise_coords(test_coords);
        % goon=input("is this ok?");
        pause(3);
    end
end

%% IK looper tester
test_points=[-0.025,-0.175,0.01];
for i=1:20
    joints=arm.get_joint_states(test_points);
    arm.visualise_motion(joints);
    pause(5);
end

%% CHomp tester
test_points=[0.025,-0.15,0.01];
for i=1:3
    [final_joints, waypoints,times]=arm.dochomp(test_points, 5, 1);
    arm.visualise_motion(final_joints);
    pause(5);
    arm.visualise_motion_chomp(waypoints)
    pause(5);
end

%% my plan tester
test_points=[-0.025,-0.175,0.01];
for i=1:20
    [final_joints, waypoints,times]=arm.dochomp(test_points, 5, 0.1);
    arm.visualise_motion_chomp(waypoints)
    pause(5);
    n=length(times);
    indices=[1 int32(n/4) int32(n/2) int32(3*n/4) n];
    for j=1:length(indices)
        trajectory(j,:)=waypoints(indices(j),:);
    end
    arm.visualise_motion(trajectory(1,:));
    pause(3);
    arm.visualise_motion(trajectory(2,:));
    pause(3);
    arm.visualise_motion(trajectory(3,:));
    pause(3);
    arm.visualise_motion(trajectory(4,:));
    pause(3);
    arm.visualise_motion(trajectory(5,:));
    pause(3);
end

%% Reading all the coords
chess_joints=zeros(8,4,8);
for i=1:8
    for j=1:8
        [arm, joint_vals]=robotarm;
        chess_joints(j,:,i)=joint_vals(:);
        disp("Joints have been recorded");
        disp("Change now, time limit is 20 secs");
        waiter=input("Press Enter when done");
    end
end

%% forward kinematics




%% chessboard points extrapolator

