% code to run the joints version of the robot arm control

joint_controller=robotarm_joints;
game_over=false;
move_out_joints=[-90 90 45 0];
while(~game_over)

    SERVER = 'http://10.205.38.238:5000/move';

    try
        % Send the GET request and decode the JSON response
        response = webread(SERVER);
    
        % Assuming response is a structure equivalent to your 'move' dictionary
        disp('Request successful.');
        disp('Received move:');
        disp(response);
    
        % Use the response as needed
        % For example, if the response is a structure with a field 'move'
        if isfield(response, 'move')
            moveValue = response.move;
            disp(['Move is: ', moveValue]);
        else
            disp('No move found in the response.');
        end
    
    catch ME
        % Handle errors from the web request
        fprintf('Board request failed. Error message: %s\n', ME.message);
        return;
    end

    pick=[row_finder(moveValue(1)) int32(str2double(moveValue(2)))];
     place =[row_finder(moveValue(3)) int32(str2double(moveValue(4)))];

    disp("Moving out of the way")
    joint_controller.setposition(move_out_joints);
    pause(2);
    % setting the coordinates
    % pick_point=input("Enter The Pick Coordinates \n");
    % col=int32(pick_point/10);
    % row=mod(pick_point, 10);
    % if row>=5
    %     col=col-1;
    % end
    % pick_state(:)=rad2deg(chess_joints(row,:,col));

    pick_state(:)=rad2deg(chess_joints(pick(1),:,pick(2)));

    %setting palce coordinates
    % place_point=input("Enter The Pick Coordinates \n");
    % col=int32(floor(place_point/10));
    % row=mod(place_point, 10);
    %  if row>=5
    %     col=col-1;
    % end
    % place_state(:)=rad2deg(chess_joints(row,:,col));

    pick_state(:)=rad2deg(chess_joints(place(1),:,place(2)));

    disp("Starting picking");
    joint_controller.setposition(pick_state);
    pause(5);
    disp("Switching on the Magnet");
    joint_controller.magnet(1);
    disp("Going back to home");
    joint_controller.set_to_home();
    pause(5);
    disp("Going to Place point");
    joint_controller.setposition(place_state);
    pause(5);
    disp("Switching off the Magnet");
    joint_controller.magnet(0);
    disp("Heading Back home");
    joint_controller.set_to_home();
    pause(5);
    disp("Moving out of the way")
    joint_controller.setposition(move_out_joints);
    pause(5);

    game_over=true;
    % % Uncomment this for the live game then you can press 0 each time
    % game_over=input("Next Move? \n");

end

%%
% row=row_finder(place(1))
function row_val=row_finder(pos)
    switch pos
        case 'a'
            row_val=8;
        case 'b'
            row_val=7;
        case 'c'
            row_val=6;
        case 'd'
            row_val=5;
        case 'e'
            row_val=4;
        case 'f'
            row_val=3;
        case 'g'
            row_val=2;
        case 'h'
            row_val=1;
    end
end