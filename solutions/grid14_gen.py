import os
base = '/home/user/MRPC-2025-homework/solutions/variants'
os.makedirs(base, exist_ok=True)
list_path = os.path.join(base, 'grid14_list.txt')

variants = []

def write_variant(name, overrides):
    var_dir = os.path.join(base, name)
    os.makedirs(var_dir, exist_ok=True)
    content = (
        f"# grid14 turn-based time scaling\n"
        f"search_mode={overrides.get('search_mode','theta_star')}\n"
        f"heuristic_weight={overrides.get('heuristic_weight',1.5)}\n"
        f"ara_init_weight={overrides.get('ara_init_weight',1.6)}\n"
        f"ara_step={overrides.get('ara_step',0.2)}\n"
        f"ara_max_iter={overrides.get('ara_max_iter',3)}\n"
        f"z_penalty={overrides.get('z_penalty',0.10)}\n"
        f"theta_rewire_z_penalty={overrides.get('theta_rewire_z_penalty',0.0)}\n"
        f"time_vel_scale={overrides.get('time_vel_scale',1.15)}\n"
        f"time_acc_scale={overrides.get('time_acc_scale',1.05)}\n"
        f"time_global_scale={overrides.get('time_global_scale',0.92)}\n"
        f"path_simplify_scale={overrides.get('path_simplify_scale',1.0)}\n"
        f"shortcut_passes={overrides.get('shortcut_passes',1)}\n"
        f"los_step={overrides.get('los_step',0.10)}\n"
        f"straight_time_scale={overrides.get('straight_time_scale',1.0)}\n"
        f"turn_time_scale={overrides.get('turn_time_scale',1.0)}\n"
        f"turn_angle_deg={overrides.get('turn_angle_deg',90.0)}\n"
    )
    with open(os.path.join(var_dir, 'variant_override.txt'), 'w') as f:
        f.write(content)
    variants.append(name)

write_variant('variant_G14_s80_t100_a25', {'straight_time_scale':0.8, 'turn_time_scale':1.0, 'turn_angle_deg':25})
write_variant('variant_G14_s70_t100_a25', {'straight_time_scale':0.7, 'turn_time_scale':1.0, 'turn_angle_deg':25})
write_variant('variant_G14_s80_t110_a25', {'straight_time_scale':0.8, 'turn_time_scale':1.1, 'turn_angle_deg':25})
write_variant('variant_G14_s70_t110_a25', {'straight_time_scale':0.7, 'turn_time_scale':1.1, 'turn_angle_deg':25})
write_variant('variant_G14_s80_t100_a35', {'straight_time_scale':0.8, 'turn_time_scale':1.0, 'turn_angle_deg':35})
write_variant('variant_G14_s70_t110_a35', {'straight_time_scale':0.7, 'turn_time_scale':1.1, 'turn_angle_deg':35})

with open(list_path, 'w') as f:
    f.write("\n".join(variants))

print(f"Generated {len(variants)} variants -> {list_path}")
