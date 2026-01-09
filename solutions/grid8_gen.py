import os, itertools
base = '/home/user/MRPC-2025-homework/solutions/variants'
os.makedirs(base, exist_ok=True)
list_path = os.path.join(base, 'grid8_list.txt')
search_mode = 'theta_star'
heuristic_weight = 1.5
ara_init_weight = 1.6
ara_step = 0.2
ara_max_iter = 3
z_vals = [0.06, 0.03, 0.0]
los_vals = [0.15, 0.20]
path_simplify_scale = 1.0
shortcut_passes = 1
# time scales from current best
vel_s = 1.15
acc_s = 1.05
t_global = 0.92
names = []
for z_penalty, los_step in itertools.product(z_vals, los_vals):
    name = f"variant_G8_z{int(round(z_penalty*100)):02d}_l{int(los_step*100):02d}"
    var_dir = os.path.join(base, name)
    os.makedirs(var_dir, exist_ok=True)
    content = (
        f"# grid8 z/los aggressive\n"
        f"search_mode={search_mode}\n"
        f"heuristic_weight={heuristic_weight}\n"
        f"ara_init_weight={ara_init_weight}\n"
        f"ara_step={ara_step}\n"
        f"ara_max_iter={ara_max_iter}\n"
        f"z_penalty={z_penalty}\n"
        f"theta_rewire_z_penalty=0.0\n"
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
