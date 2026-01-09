import os, itertools
base = '/home/user/MRPC-2025-homework/solutions/variants'
os.makedirs(base, exist_ok=True)
list_path = os.path.join(base, 'grid9_list.txt')
search_mode = 'theta_star'
heuristic_weight = 1.5
ara_init_weight = 1.6
ara_step = 0.2
ara_max_iter = 3
z_penalty = 0.10
theta_rewire_z_penalty = 0.0
los_step = 0.10
# time scales from current best
vel_s = 1.15
acc_s = 1.05
t_global = 0.92
path_res_vals = [0.25, 0.30, 0.35]
path_simplify_vals = [1.5, 2.0]
shortcut_vals = [10, 20]

names = []
for pr, ps, sp in itertools.product(path_res_vals, path_simplify_vals, shortcut_vals):
    name = f"variant_G9_r{int(pr*100):02d}_p{int(ps*100):03d}_s{sp:02d}"
    var_dir = os.path.join(base, name)
    os.makedirs(var_dir, exist_ok=True)
    content = (
        f"# grid9 path resolution + simplify\n"
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
        f"path_simplify_scale={ps}\n"
        f"shortcut_passes={sp}\n"
        f"los_step={los_step}\n"
        f"path_resolution={pr}\n"
    )
    with open(os.path.join(var_dir, 'variant_override.txt'), 'w') as f:
        f.write(content)
    names.append(name)

with open(list_path, 'w') as f:
    f.write("\n".join(names))

print(f"Generated {len(names)} variants -> {list_path}")
