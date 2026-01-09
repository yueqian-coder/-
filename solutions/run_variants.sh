#!/bin/bash
set -u
export DISPLAY=:0
export XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir

repo=/home/user/MRPC-2025-homework
variants_dir="$repo/solutions/variants"
active_override="$repo/solutions/variant_override.txt"

mkdir -p /opt/focal/home/user
if ! mountpoint -q /opt/focal/home/user; then
  mount --bind /home/user /opt/focal/home/user
fi

mkdir -p /opt/focal/tmp/.X11-unix
if ! mountpoint -q /opt/focal/tmp/.X11-unix; then
  mount --bind /tmp/.X11-unix /opt/focal/tmp/.X11-unix
fi

mkdir -p /opt/focal/mnt/wslg
if ! mountpoint -q /opt/focal/mnt/wslg; then
  mount --bind /mnt/wslg /opt/focal/mnt/wslg
fi

if [ -n "${VARIANTS:-}" ]; then
  IFS=',' read -r -a variants <<< "$VARIANTS"
else
  variants=(
    variant_A_baseline
    variant_B_fast_time
    variant_C_more_shortcut
    variant_D_lazy_theta
    variant_E_weighted
    variant_F_ara
    variant_G_poly5
    variant_H_replan_high
    variant_I_low_zpen
    variant_J_weighted_fast1
    variant_K_weighted_fast2
    variant_L_weighted_slow
    variant_M_weighted_astar
    variant_N_theta_weighted_smooth
    variant_O_time_w1
    variant_P_time_w2
    variant_Q_time_w3
    variant_R_time_w4
    variant_S_time_w5
    variant_T_time_w6
    variant_U_theta_z1
    variant_V_theta_z2
    variant_W_theta_z3
    variant_X_theta_z4
    variant_Y_theta_z5
    variant_Z_theta_z6
    variant_WA_weighted_z1
    variant_WB_weighted_z2
    variant_AA_ara_fast1
    variant_AB_ara_fast2
    variant_AC_ara_fast3
    variant_AD_ara_safe1
    variant_AE_ara_safe2
    variant_AF_ara_safe3
  )
fi

summary="$variants_dir/summary.txt"
: > "$summary"

for variant in "${variants[@]}"; do
  cp "$variants_dir/$variant/variant_override.txt" "$active_override"

  timeout --signal=TERM --kill-after=10s 140s chroot /opt/focal /usr/bin/env DISPLAY=:0 XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir QT_QPA_PLATFORM=xcb /bin/bash -lc '/tmp/run_eval.sh'
  rc=$?
  if [ $rc -ne 0 ]; then
    if [ $rc -eq 124 ]; then
      echo "$variant: FAILED (run_eval timeout)" >> "$summary"
    else
      echo "$variant: FAILED (run_eval rc=$rc)" >> "$summary"
    fi
    continue
  fi

  data="$repo/code/src/quadrotor_simulator/so3_control/src/control_data.txt"
  timedata="$repo/code/src/quadrotor_simulator/so3_control/src/control_timedata.txt"

  if [ ! -f "$data" ] || [ ! -f "$timedata" ]; then
    echo "$variant: FAILED (missing data)" >> "$summary"
    continue
  fi

  if ! python3 "$repo/code/calculate_results.py" >/tmp/variant_score.log; then
    echo "$variant: FAILED (score)" >> "$summary"
    continue
  fi

  cp "$repo/solutions/result.txt" "$variants_dir/$variant/result.txt"
  cp "$data" "$variants_dir/$variant/control_data.txt"
  cp "$timedata" "$variants_dir/$variant/control_timedata.txt"
  echo "$variant: $(cat "$repo/solutions/result.txt")" >> "$summary"

done
