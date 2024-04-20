arm=robotarm;

pick_coords=[-0.1 -0.1 0.2];
place_coords=[-0.2 -0.1 0.1];
%%
arm.dochomp(pick_coords, 3, 0.1);
arm.set_to_home();
arm.dochomp(place_coords,3, 0.1);
arm.set_to_home();