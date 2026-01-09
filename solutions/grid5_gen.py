import os, itertools
base = '/home/user/MRPC-2025-homework/solutions/variants'
os.makedirs(base, exist_ok=True)
list_path = os.path.join(base, 'grid5_list.txt')
search_mode = 'theta_star'
heuristic_weight = 1.5
ara_init_weight = 1.6
ara_step = 0.2
ara_max_iter = 3
z_penalty = 0.1
theta_rewire_z_penalty = 0.0
los_step = 0.10
path_simplify_scale = 1.0
shortcut_passes = 1
vel_vals = [1.0, 1.1]
acc_vals = [0.9, 1.0]
kg_vals = [1.05, 1.10, 1.15]
names = []
for vel_s, acc_s, t_global in itertools.product(vel_vals, acc_vals, kg_vals):
    name = f"variant_G5_v{int(vel_s*100):03d}_a{int(acc_s*100):03d}_t{int(t_global*100):03d}"
    var_dir = os.path.join(base, name)
    os.makedirs(var_dir, exist_ok=True)
    content = (
        f"# grid5 slower time scales\n"
        f"search_mode={search_mode}\n"
        f"heuristic_weight={heuristic_weight}\n"
        f"ara_init_weight={ara_init_weight}\n"
        f"ara_step={ara_step}\n"
        f"ara_max_iter={ara_max_iter}\n"
        f"z_penalty={z_penalty}\n"
        f"theta_rewire_z_penalty={theta_rewire_z_penalty}\n"
        f"time_vel_scale={vel_s}\n"
        f"time_acc_scale={acc_s}\n"
        f"time_global_scale={t_global}\n"
        f"path_simplify_scale={path_simplify_scale}\n"
        f"shortcut_passes={shortcut_passes}\n"
        f"los_step={los_step}\n"
    )
    with open(os.path.join(var_dir, 'variant_override.txt'), 'w') as f:
        f.write(content)
    names.append(name)

with open(list_path, 'w') as f:
    f.write("\n".join(names))

print(f"Generated {len(names)} variants -> {list_path}")
