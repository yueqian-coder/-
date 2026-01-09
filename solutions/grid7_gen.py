import os
base = '/home/user/MRPC-2025-homework/solutions/variants'
os.makedirs(base, exist_ok=True)
list_path = os.path.join(base, 'grid7_list.txt')
# time scales from current best (variant_G1_z10_l10_t92)
base_time = {
    'time_vel_scale': 1.15,
    'time_acc_scale': 1.05,
    'time_global_scale': 0.92,
}
common = {
    'search_mode': 'theta_star',
    'heuristic_weight': 1.5,
    'ara_init_weight': 1.6,
    'ara_step': 0.2,
    'ara_max_iter': 3,
    'z_penalty': 0.10,
    'theta_rewire_z_penalty': 0.0,
    'los_step': 0.10,
    'path_simplify_scale': 1.0,
    'shortcut_passes': 1,
}
common.update(base_time)

variants = []
for ps in (1.2, 1.5, 2.0):
    for sp in (5, 10, 20):
        name = f"variant_G7_p{int(ps*100):03d}_s{sp:02d}"
        var_dir = os.path.join(base, name)
        os.makedirs(var_dir, exist_ok=True)
        content = (
            f"# grid7 simplify/shortcut\n"
            f"search_mode={common['search_mode']}\n"
            f"heuristic_weight={common['heuristic_weight']}\n"
            f"ara_init_weight={common['ara_init_weight']}\n"
            f"ara_step={common['ara_step']}\n"
            f"ara_max_iter={common['ara_max_iter']}\n"
            f"z_penalty={common['z_penalty']}\n"
            f"theta_rewire_z_penalty={common['theta_rewire_z_penalty']}\n"
            f"time_vel_scale={common['time_vel_scale']}\n"
            f"time_acc_scale={common['time_acc_scale']}\n"
            f"time_global_scale={common['time_global_scale']}\n"
            f"path_simplify_scale={ps}\n"
            f"shortcut_passes={sp}\n"
            f"los_step={common['los_step']}\n"
        )
        with open(os.path.join(var_dir, 'variant_override.txt'), 'w') as f:
            f.write(content)
        variants.append(name)

with open(list_path, 'w') as f:
    f.write("\n".join(variants))

print(f"Generated {len(variants)} variants -> {list_path}")
