import os
base = '/home/user/MRPC-2025-homework/solutions/variants'
os.makedirs(base, exist_ok=True)
list_path = os.path.join(base, 'grid12_list.txt')

variants = []

def write_variant(name, overrides):
    var_dir = os.path.join(base, name)
    os.makedirs(var_dir, exist_ok=True)
    content = (
        f"# grid12 dev_order\n"
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
        f"dev_order={overrides.get('dev_order',3)}\n"
        f"min_order={overrides.get('min_order',3)}\n"
    )
    with open(os.path.join(var_dir, 'variant_override.txt'), 'w') as f:
        f.write(content)
    variants.append(name)

write_variant('variant_G12_d4', {'dev_order':4, 'min_order':4})
write_variant('variant_G12_d5', {'dev_order':5, 'min_order':5})
write_variant('variant_G12_d4_smooth', {'dev_order':4, 'min_order':4, 'path_simplify_scale':1.2, 'shortcut_passes':5})
write_variant('variant_G12_d5_smooth', {'dev_order':5, 'min_order':5, 'path_simplify_scale':1.2, 'shortcut_passes':5})

with open(list_path, 'w') as f:
    f.write("\n".join(variants))

print(f"Generated {len(variants)} variants -> {list_path}")
