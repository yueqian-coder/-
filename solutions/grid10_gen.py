import os, itertools
base = '/home/user/MRPC-2025-homework/solutions/variants'
os.makedirs(base, exist_ok=True)
list_path = os.path.join(base, 'grid10_list.txt')
search_mode = 'theta_star'
heuristic_weight = 1.5
ara_init_weight = 1.6
ara_step = 0.2
ara_max_iter = 3
z_penalty = 0.10
theta_rewire_z_penalty = 0.0
los_step = 0.10
use_los_shortest = 1

variants = []

def write_variant(name, vel_s, acc_s, t_global, ps, sp):
    var_dir = os.path.join(base, name)
    os.makedirs(var_dir, exist_ok=True)
    content = (
        f"# grid10 los-shortest\n"
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
        f"use_los_shortest={use_los_shortest}\n"
    )
    with open(os.path.join(var_dir, 'variant_override.txt'), 'w') as f:
        f.write(content)
    variants.append(name)

for ps, sp in itertools.product([1.0, 1.5], [5, 10]):
    name = f"variant_G10_base_p{int(ps*100):03d}_s{sp:02d}"
    write_variant(name, 1.15, 1.05, 0.92, ps, sp)

for ps, sp in itertools.product([1.0, 1.5], [5, 10]):
    name = f"variant_G10_fast_p{int(ps*100):03d}_s{sp:02d}"
    write_variant(name, 1.25, 1.15, 0.88, ps, sp)

with open(list_path, 'w') as f:
    f.write("\n".join(variants))

print(f"Generated {len(variants)} variants -> {list_path}")
