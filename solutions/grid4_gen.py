import os
base = '/home/user/MRPC-2025-homework/solutions/variants'
os.makedirs(base, exist_ok=True)
variants = []

def write_variant(name, content):
    var_dir = os.path.join(base, name)
    os.makedirs(var_dir, exist_ok=True)
    with open(os.path.join(var_dir, 'variant_override.txt'), 'w') as f:
        f.write(content)
    variants.append(name)

# Common baseline
common = {
    'search_mode': 'theta_star',
    'heuristic_weight': 1.5,
    'ara_init_weight': 1.6,
    'ara_step': 0.2,
    'ara_max_iter': 3,
    'z_penalty': 0.1,
    'theta_rewire_z_penalty': 0.0,
    'time_vel_scale': 1.15,
    'time_acc_scale': 1.05,
    'time_global_scale': 0.92,
    'los_step': 0.10,
}

def make_content(overrides):
    data = dict(common)
    data.update(overrides)
    return (
        f"# grid4 other methods\n"
        f"search_mode={data['search_mode']}\n"
        f"heuristic_weight={data['heuristic_weight']}\n"
        f"ara_init_weight={data['ara_init_weight']}\n"
        f"ara_step={data['ara_step']}\n"
        f"ara_max_iter={data['ara_max_iter']}\n"
        f"z_penalty={data['z_penalty']}\n"
        f"theta_rewire_z_penalty={data['theta_rewire_z_penalty']}\n"
        f"time_vel_scale={data['time_vel_scale']}\n"
        f"time_acc_scale={data['time_acc_scale']}\n"
        f"time_global_scale={data['time_global_scale']}\n"
        f"path_simplify_scale={data['path_simplify_scale']}\n"
        f"shortcut_passes={data['shortcut_passes']}\n"
        f"los_step={data['los_step']}\n"
    )

# Path optimization: shortcut passes + simplify scale
for sp in (2, 3):
    for ps in (0.9, 1.0, 1.1):
        name = f"variant_G4_s{sp}_p{int(ps*100):03d}"
        content = make_content({
            'path_simplify_scale': ps,
            'shortcut_passes': sp,
        })
        write_variant(name, content)

# Lazy theta variants
lazy_cases = [
    ('variant_G4_lazy_1', {'search_mode': 'lazy_theta_star', 'path_simplify_scale': 1.0, 'shortcut_passes': 1}),
    ('variant_G4_lazy_2', {'search_mode': 'lazy_theta_star', 'los_step': 0.08, 'path_simplify_scale': 1.0, 'shortcut_passes': 1}),
    ('variant_G4_lazy_3', {'search_mode': 'lazy_theta_star', 'time_global_scale': 0.90, 'path_simplify_scale': 1.0, 'shortcut_passes': 1}),
]
for name, ov in lazy_cases:
    content = make_content(ov)
    write_variant(name, content)

# Weighted A* variants
weighted_cases = [
    ('variant_G4_weighted_1', {'search_mode': 'weighted', 'path_simplify_scale': 1.0, 'shortcut_passes': 1}),
    ('variant_G4_weighted_2', {'search_mode': 'weighted', 'time_global_scale': 0.90, 'path_simplify_scale': 1.1, 'shortcut_passes': 1}),
]
for name, ov in weighted_cases:
    content = make_content(ov)
    write_variant(name, content)

# Theta rewire z penalty variants
rewire_cases = [
    ('variant_G4_theta_zrewire_1', {'theta_rewire_z_penalty': 0.1, 'path_simplify_scale': 1.0, 'shortcut_passes': 1}),
    ('variant_G4_theta_zrewire_2', {'theta_rewire_z_penalty': 0.2, 'path_simplify_scale': 1.0, 'shortcut_passes': 1}),
]
for name, ov in rewire_cases:
    content = make_content(ov)
    write_variant(name, content)

list_path = os.path.join(base, 'grid4_list.txt')
with open(list_path, 'w') as f:
    f.write("\n".join(variants))

print(f"Generated {len(variants)} variants -> {list_path}")
