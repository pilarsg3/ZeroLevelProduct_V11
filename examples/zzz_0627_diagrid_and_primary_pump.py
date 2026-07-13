from ocp_vscode import show
from assemble import assemble_objects

diagrid = {
    "obj_type":      "diagrid",
    "obj_id":        "diagrid",
    "diameter":      4.660,
    "height":        1.050,
    "z_bottom":      -0.460,
    "wall_t_side":   0.030,
    "wall_t_top":    0.030,
    "wall_t_bottom": 0.030,
    "boss_wall_t":   0.071,
}

pump = {
    "obj_type":      "primary_pump",
    "obj_id":        "pump",
    "barrel_radius":  0.650,
    "barrel_wall_t":  0.040,
    "barrel_height":  10.0,
    "nozzle_r_pipe":  0.220,
    "nozzle_wall_t":  0.025,
    "nozzle_L_leg":   0.600,
    "nozzle_R_bend":  0.440,
    "nozzle_arc_deg": 105.0,
    "nozzle_L_inlet": 0.050,
    "nozzle_z":       0.450,
    "flange_width":   0.520,
    "flange_height":  0.850,
    "flange_depth":   0.480,
    "at_radius":      3.500,
    "at_angle_deg":   0.0,
}

#show(assemble_objects([diagrid, pump]))
show(assemble_objects([diagrid]))
