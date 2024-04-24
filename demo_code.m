% code to run the joints version of the robot arm control
joint_controller=robotarm_joints;
game_over=false;
while(~game_over)
    % setting the coordinates
    pick_point=input("Enter The Pick Coordinates \n");
    col=int32(pick_point/10)
    row=mod(pick_point, 10)
    pick_state(:)=rad2deg(joint_controller.states(row, :, col));

    %setting palce coordinates
    place_point=input("Enter The Pick Coordinates \n");
    col=int32(place_point/10)
    row=mod(place_point, 10)
    place_state(:)=rad2deg(joint_controller.states(row, :, col));

    disp("Starting picking");
    joint_controller.setposition(pick_state);
    pause(4);
    disp("Switching on the Magnet");
    joint_controller.magnet(1);
    disp("Going back to home");
    joint_controller.set_to_home();
    pause(4);
    disp("Going to Place point");
    joint_controller.setposition(place_state);
    pause(4);
    disp("Switching off the Magnet");
    joint_controller.magnet(0);
    disp("Heading Back home");
    joint_controller.set_to_home();
    pause(4);

    game_over=true;
    % % Uncomment this for the live game then you can press 0 each time
    % game_over=input("Next Move? \n");

end

function picknplace=get_pick_place()
    robot_client=tcpclient(address, port);
    in=read(robot_client, length, "string");
    picknplace=str2double(in);
end