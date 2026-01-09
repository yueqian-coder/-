import os
base = '/home/user/MRPC-2025-homework/solutions/variants'
os.makedirs(base, exist_ok=True)
list_path = os.path.join(base, 'grid11_list.txt')

variants = []

def write_variant(name, overrides):
    var_dir = os.path.join(base, name)
    os.makedirs(var_dir, exist_ok=True)
    content = (
        f"# grid11 raw occ\n"
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
        f"use_los_shortest={overrides.get('use_los_shortest',0)}\n"
        f"use_raw_occ={overrides.get('use_raw_occ',1)}\n"
    )
    with open(os.path.join(var_dir, 'variant_override.txt'), 'w') as f:
        f.write(content)
    variants.append(name)

write_variant('variant_G11_raw_base', {})
write_variant('variant_G11_raw_los', {'use_los_shortest':1})
write_variant('variant_G11_raw_smooth', {'path_simplify_scale':1.5, 'shortcut_passes':5})
write_variant('variant_G11_raw_los_smooth', {'use_los_shortest':1, 'path_simplify_scale':1.5, 'shortcut_passes':5})
write_variant('variant_G11_raw_fast1', {'use_los_shortest':1, 'time_vel_scale':1.25, 'time_acc_scale':1.15, 'time_global_scale':0.88})
write_variant('variant_G11_raw_fast2', {'use_los_shortest':1, 'time_vel_scale':1.35, 'time_acc_scale':1.25, 'time_global_scale':0.84})

with open(list_path, 'w') as f:
    f.write("\n".join(variants))

print(f"Generated {len(variants)} variants -> {list_path}")
