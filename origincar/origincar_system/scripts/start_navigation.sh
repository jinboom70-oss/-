#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="${WORKSPACE:-/userdata/dev_ws}"
SETUP_FILE="${SETUP_FILE:-${WORKSPACE}/install/setup.bash}"
MAP_FILE="${MAP_FILE:-${WORKSPACE}/src/origincar/map/race_modify.yaml}"                     #主地图 YAML 文件，包含环境地图和导航相关配置。
KEEPOUT_MAP_FILE="${KEEPOUT_MAP_FILE:-${WORKSPACE}/src/origincar/map/race_keepout.yaml}"    #keepout 虚拟障碍物地图 YAML 文件。
WAYPOINTS_FILE="${WAYPOINTS_FILE:-${WORKSPACE}/src/origincar/origincar_system/config/waypoints.yaml}"#多航点配置文件，包含顺时针/逆时针路线。
RVIZ_CONFIG_FILE="${RVIZ_CONFIG_FILE:-${WORKSPACE}/src/origincar/default.rviz}"

RVIZ_DELAY="${RVIZ_DELAY:-3}"
BRINGUP_DELAY="${BRINGUP_DELAY:-5}"
NAV2_DELAY="${NAV2_DELAY:-15}"
QR_DELAY="${QR_DELAY:-0}"

START_CAMERA="${START_CAMERA:-1}"                     # START_CAMERA：是否启动 USB 相机；1=启动，0=不启动。
START_QR_DETECTOR="${START_QR_DETECTOR:-1}"           # START_QR_DETECTOR：是否启动二维码识别节点；1=启动，0=不启动。
START_IMAGE_ANALYZER="${START_IMAGE_ANALYZER:-0}"     # START_IMAGE_ANALYZER：是否启动阿里云图像识别节点；未配置 API key 时建议设为 0。
START_MULTI_POINT="${START_MULTI_POINT:-0}"           # START_MULTI_POINT：是否启动多点巡航；0 时只启动基础节点，小车不会自动跑。
WAIT_FOR_NAV_START="${WAIT_FOR_NAV_START:-1}"         # WAIT_FOR_NAV_START：启动巡航前是否等待用户按 Enter，方便先在 RViz 设置初始位姿。

NAV_MODE="${NAV_MODE:-path_follower}"
START_FULL_NAV2="${START_FULL_NAV2:-1}"               # START_FULL_NAV2：是否启动完整 Nav2；0=只启动地图/AMCL/keepout 显示，1=启动完整 Nav2。
ENABLE_QR_SKIP_FIRST="${ENABLE_QR_SKIP_FIRST:-true}"  # ENABLE_QR_SKIP_FIRST：到 1 号点途中识别到二维码后，是否直接切到 2 号点倒车前往。

QR_TOPIC="${QR_TOPIC:-/qr_info}"                      # QR_TOPIC：二维码识别结果发布话题。
ROUTE_DIRECTION="${ROUTE_DIRECTION:-auto}"            # ROUTE_DIRECTION：默认路线方向；auto=等待二维码决定，clockwise=顺时针，counterclockwise=逆时针。

ENABLE_QR_DIRECTION_SELECT="${ENABLE_QR_DIRECTION_SELECT:-true}"   # ENABLE_QR_DIRECTION_SELECT：是否允许二维码选择顺时针/逆时针路线。
USE_QR_PARITY_DIRECTION="${USE_QR_PARITY_DIRECTION:-true}"         # USE_QR_PARITY_DIRECTION：是否使用二维码奇偶规则选方向；奇数=顺时针，偶数=逆时针。
CLOCKWISE_QR_VALUE="${CLOCKWISE_QR_VALUE:-1}"                      # CLOCKWISE_QR_VALUE：不使用奇偶规则时，代表顺时针的二维码数值。
COUNTERCLOCKWISE_QR_VALUE="${COUNTERCLOCKWISE_QR_VALUE:-2}"        # COUNTERCLOCKWISE_QR_VALUE：不使用奇偶规则时，代表逆时针的二维码数值。


QR_IMAGE_TOPIC="${QR_IMAGE_TOPIC:-/image}"   # QR_IMAGE_TOPIC：二维码识别使用的相机图像话题。


QR_SCAN_HZ="${QR_SCAN_HZ:-30.0}"      # QR_SCAN_HZ：二维码识别频率，单位 Hz。


QR_DETECTOR_ENGINE="${QR_DETECTOR_ENGINE:-opencv}"    # QR_DETECTOR_ENGINE：二维码识别后端；当前默认 opencv。


QR_RESIZE_WIDTH="${QR_RESIZE_WIDTH:-0}"       # QR_RESIZE_WIDTH：二维码识别前图像缩放宽度；0 表示不缩放。

       
QR_REPEAT_PUBLISH_INTERVAL="${QR_REPEAT_PUBLISH_INTERVAL:-5.0}"# QR_REPEAT_PUBLISH_INTERVAL：同一个二维码重复发布的最小间隔，单位秒。

IMAGE_ANALYSIS_WAYPOINT="${IMAGE_ANALYSIS_WAYPOINT:-7}"       # IMAGE_ANALYSIS_WAYPOINT：到达该航点时触发拍照识别；当前为 7 号点。


IMAGE_ANALYSIS_TRIGGER_TOPIC="${IMAGE_ANALYSIS_TRIGGER_TOPIC:-/person_trigger}"   # IMAGE_ANALYSIS_TRIGGER_TOPIC：触发图像识别节点拍照/分析的话题。


IMAGE_ANALYSIS_RESULT_TOPIC="${IMAGE_ANALYSIS_RESULT_TOPIC:-/image_ai}"          # IMAGE_ANALYSIS_RESULT_TOPIC：图像识别结果发布话题。


IMAGE_ANALYSIS_IMAGE_TOPIC="${IMAGE_ANALYSIS_IMAGE_TOPIC:-/image}"            # IMAGE_ANALYSIS_IMAGE_TOPIC：图像识别使用的相机图像话题。


IMAGE_ANALYSIS_IMAGE_MSG_TYPE="${IMAGE_ANALYSIS_IMAGE_MSG_TYPE:-compressed}"     # IMAGE_ANALYSIS_IMAGE_MSG_TYPE：图像消息类型；compressed=压缩图，raw=原始图。

case "${ENABLE_QR_SKIP_FIRST}" in
  1|true|TRUE|True|yes|YES|Yes|on|ON|On)
    ENABLE_QR_SKIP_FIRST="true"
    ;;
  0|false|FALSE|False|no|NO|No|off|OFF|Off)
    ENABLE_QR_SKIP_FIRST="false"
    ;;
esac

# PASS_RADIUS：普通航点切点距离，单位 m；小车距当前航点小于该值就切到下一点。
PASS_RADIUS="${PASS_RADIUS:-0.45}"

# LOOKAHEAD_DISTANCE：前瞻距离，单位 m；越大转弯越提前、轨迹越圆滑。
LOOKAHEAD_DISTANCE="${LOOKAHEAD_DISTANCE:-0.65}"

# ENABLE_SMOOTH_KEEPOUT_PATH：是否在指定航点范围内使用平滑曲线追踪点，并参考 keepout 选择更安全的切弯点。
ENABLE_SMOOTH_KEEPOUT_PATH="${ENABLE_SMOOTH_KEEPOUT_PATH:-true}"

# SMOOTH_PATH_MIN_WAYPOINT：启用平滑 keepout 路径的起始航点编号，按 1 开始计数。
SMOOTH_PATH_MIN_WAYPOINT="${SMOOTH_PATH_MIN_WAYPOINT:-4}"

# SMOOTH_PATH_MAX_WAYPOINT：启用平滑 keepout 路径的结束航点编号，按 1 开始计数。
SMOOTH_PATH_MAX_WAYPOINT="${SMOOTH_PATH_MAX_WAYPOINT:-9}"

# SMOOTH_PATH_SAMPLES_PER_SEGMENT：每两个航点之间生成的曲线采样点数量；越大越细腻，计算量也略高。
SMOOTH_PATH_SAMPLES_PER_SEGMENT="${SMOOTH_PATH_SAMPLES_PER_SEGMENT:-10}"

# SMOOTH_PATH_KEEPOUT_CLEARANCE：平滑路径候选点期望远离 keepout 的距离，单位 m。
SMOOTH_PATH_KEEPOUT_CLEARANCE="${SMOOTH_PATH_KEEPOUT_CLEARANCE:-0.22}"

# SMOOTH_PATH_KEEPOUT_WEIGHT：keepout 距离在平滑路径选点中的权重；越大越偏向远离虚拟障碍物。
SMOOTH_PATH_KEEPOUT_WEIGHT="${SMOOTH_PATH_KEEPOUT_WEIGHT:-0.9}"

# ENABLE_SKIP_OVERSHOT_WAYPOINTS：是否启用冲过头跳点逻辑。
ENABLE_SKIP_OVERSHOT_WAYPOINTS="${ENABLE_SKIP_OVERSHOT_WAYPOINTS:-true}"

# OVERSHOT_WAYPOINT_MARGIN：判定“下一个点更近”的距离余量，单位 m。
OVERSHOT_WAYPOINT_MARGIN="${OVERSHOT_WAYPOINT_MARGIN:-0.05}"

# OVERSHOT_WAYPOINT_MAX_DISTANCE：当前未到达点允许被跳过的最大距离，单位 m。
OVERSHOT_WAYPOINT_MAX_DISTANCE="${OVERSHOT_WAYPOINT_MAX_DISTANCE:-0.80}"

# OVERSHOT_NEXT_WAYPOINT_MAX_DISTANCE：小车距离下一个点小于该值时才允许跳过当前点，单位 m。
OVERSHOT_NEXT_WAYPOINT_MAX_DISTANCE="${OVERSHOT_NEXT_WAYPOINT_MAX_DISTANCE:-0.45}"

# SKIP_OVERSHOT_MIN_WAYPOINT：允许跳点的起始航点编号，按 1 开始计数。
# 从 5 号点开始，避免 3->4 时提前把 4 号点判为冲过头而跳掉。
SKIP_OVERSHOT_MIN_WAYPOINT="${SKIP_OVERSHOT_MIN_WAYPOINT:-5}"

# SKIP_OVERSHOT_MAX_WAYPOINT：允许跳点的结束航点编号，按 1 开始计数。
SKIP_OVERSHOT_MAX_WAYPOINT="${SKIP_OVERSHOT_MAX_WAYPOINT:-9}"

# LINEAR_SPEED：通道外直线基础速度，单位 m/s。
LINEAR_SPEED="${LINEAR_SPEED:-0.85}"

# CHANNEL_LINEAR_SPEED：通道内速度，单位 m/s。
CHANNEL_LINEAR_SPEED="${CHANNEL_LINEAR_SPEED:-0.65}"

# CHANNEL_WAYPOINT_RANGES：使用通道速度的航点范围；当前 3-10 为通道内。
CHANNEL_WAYPOINT_RANGES="${CHANNEL_WAYPOINT_RANGES:-3-10}"

# MAX_ANGULAR_Z：最大转向角速度，单位 rad/s。
MAX_ANGULAR_Z="${MAX_ANGULAR_Z:-1.6}"

# TURN_ANGULAR_GAIN：前进时转向增益；越大转向越积极。
TURN_ANGULAR_GAIN="${TURN_ANGULAR_GAIN:-1.2}"

# TURN_MIN_SPEED_SCALE：前进转弯时最低线速度比例，避免转弯时完全降速。
TURN_MIN_SPEED_SCALE="${TURN_MIN_SPEED_SCALE:-0.25}"

# REVERSE_ANGULAR_GAIN：倒车前往 2 号点时的转向增益。
REVERSE_ANGULAR_GAIN="${REVERSE_ANGULAR_GAIN:-1.1}"

# REVERSE_MIN_SPEED_SCALE：倒车转弯时最低线速度比例。
REVERSE_MIN_SPEED_SCALE="${REVERSE_MIN_SPEED_SCALE:-0.30}"

# REVERSE_GOAL_LOOKAHEAD_DISTANCE：倒车接近目标点时的前瞻距离，单位 m。
REVERSE_GOAL_LOOKAHEAD_DISTANCE="${REVERSE_GOAL_LOOKAHEAD_DISTANCE:-0.25}"

# REVERSE_PASS_RADIUS：倒车航点切点距离，单位 m。
REVERSE_PASS_RADIUS="${REVERSE_PASS_RADIUS:-0.25}"

# ENABLE_REVERSE_APPROACH_NUDGE：是否在快到 2 号点时额外打一小段方向。
ENABLE_REVERSE_APPROACH_NUDGE="${ENABLE_REVERSE_APPROACH_NUDGE:-true}"

# REVERSE_APPROACH_NUDGE_WAYPOINT：倒车微调作用的航点编号。
REVERSE_APPROACH_NUDGE_WAYPOINT="${REVERSE_APPROACH_NUDGE_WAYPOINT:-2}"

# REVERSE_APPROACH_NUDGE_DISTANCE：距离该倒车航点小于该值时开始微调，单位 m。
REVERSE_APPROACH_NUDGE_DISTANCE="${REVERSE_APPROACH_NUDGE_DISTANCE:-0.35}"

# REVERSE_APPROACH_NUDGE_ANGULAR_Z：倒车微调角速度，单位 rad/s；正负决定打方向方向。
REVERSE_APPROACH_NUDGE_ANGULAR_Z="${REVERSE_APPROACH_NUDGE_ANGULAR_Z:--0.35}"

# OBSTACLE_ENABLE_FROM_WAYPOINT：从第几个航点开始启用雷达避障；0 表示从启动开始全程启用。
OBSTACLE_ENABLE_FROM_WAYPOINT="${OBSTACLE_ENABLE_FROM_WAYPOINT:-0}"

# OBSTACLE_STOP_DISTANCE：雷达前方障碍急停距离，单位 m。
OBSTACLE_STOP_DISTANCE="${OBSTACLE_STOP_DISTANCE:-0.35}"

# OBSTACLE_SLOW_DISTANCE：雷达前方障碍减速距离，单位 m。
OBSTACLE_SLOW_DISTANCE="${OBSTACLE_SLOW_DISTANCE:-0.90}"

# OBSTACLE_AVOID_DISTANCE：雷达前方障碍转向避让距离，单位 m。
OBSTACLE_AVOID_DISTANCE="${OBSTACLE_AVOID_DISTANCE:-0.65}"

# FRONT_ANGLE_DEG：雷达正前方检测扇区半角，单位 degree；越大检测范围越宽。
FRONT_ANGLE_DEG="${FRONT_ANGLE_DEG:-35.0}"

# ENABLE_KEEPOUT_AVOIDANCE：是否让 keepout 虚拟障碍物参与自写路径跟随器避让。
ENABLE_KEEPOUT_AVOIDANCE="${ENABLE_KEEPOUT_AVOIDANCE:-true}"

# KEEPOUT_MASK_TOPIC：keepout mask 栅格地图话题。
KEEPOUT_MASK_TOPIC="${KEEPOUT_MASK_TOPIC:-/filter_mask}"

# KEEPOUT_OCCUPIED_THRESHOLD：keepout 栅格占用阈值；大于等于该值视为虚拟障碍。
KEEPOUT_OCCUPIED_THRESHOLD="${KEEPOUT_OCCUPIED_THRESHOLD:-50}"

# KEEPOUT_STOP_DISTANCE：keepout 软约束的近距离参考值，单位 m；默认不硬停，只用于计算减速强度。
KEEPOUT_STOP_DISTANCE="${KEEPOUT_STOP_DISTANCE:-0.25}"

# KEEPOUT_SLOW_DISTANCE：距离 keepout 虚拟障碍小于该值时开始减速，单位 m。
KEEPOUT_SLOW_DISTANCE="${KEEPOUT_SLOW_DISTANCE:-0.70}"

# KEEPOUT_AVOID_DISTANCE：距离 keepout 虚拟障碍小于该值时开始转向避让，单位 m。
KEEPOUT_AVOID_DISTANCE="${KEEPOUT_AVOID_DISTANCE:-0.55}"

# KEEPOUT_SIDE_SAMPLE_DISTANCE：检测左右两侧 keepout 空间的采样距离，单位 m。
KEEPOUT_SIDE_SAMPLE_DISTANCE="${KEEPOUT_SIDE_SAMPLE_DISTANCE:-0.35}"

# ENABLE_KEEPOUT_HARD_STOP：是否把 keepout 当硬障碍停车；默认 false，避免压到 keepout 后卡死。
ENABLE_KEEPOUT_HARD_STOP="${ENABLE_KEEPOUT_HARD_STOP:-false}"

# KEEPOUT_MIN_SPEED_SCALE：keepout 软约束触发时最低保留速度比例；越大越不容易卡住。
KEEPOUT_MIN_SPEED_SCALE="${KEEPOUT_MIN_SPEED_SCALE:-0.45}"

# BACKUP_TRIGGER_TIME：雷达急停后等待多久开始倒车脱困，单位 s；0 表示立刻倒车。
BACKUP_TRIGGER_TIME="${BACKUP_TRIGGER_TIME:-0.0}"

# BACKUP_DURATION：每次倒车脱困持续时间，单位 s。
BACKUP_DURATION="${BACKUP_DURATION:-0.5}"

# BACKUP_SPEED：倒车脱困速度，单位 m/s。
BACKUP_SPEED="${BACKUP_SPEED:-0.25}"

# BACKUP_ANGULAR_Z：倒车脱困时附带的角速度，单位 rad/s。
BACKUP_ANGULAR_Z="${BACKUP_ANGULAR_Z:-0.35}"

# BACKUP_STOP_DISTANCE：后方安全距离，单位 m；后方障碍物小于该值时禁止倒车。
BACKUP_STOP_DISTANCE="${BACKUP_STOP_DISTANCE:-0.22}"

# MAX_GOAL_RETRIES：Nav2 多目标模式下单个目标失败后的最大重试次数。
MAX_GOAL_RETRIES="${MAX_GOAL_RETRIES:-2}"

# INITIAL_POSE_X：自动发布初始位姿的 x 坐标，单位 m。
INITIAL_POSE_X="${INITIAL_POSE_X:-0.655}"

# INITIAL_POSE_Y：自动发布初始位姿的 y 坐标，单位 m。
INITIAL_POSE_Y="${INITIAL_POSE_Y:-0.012}"

# INITIAL_POSE_A：自动发布初始位姿的 yaw 角，单位 rad。
INITIAL_POSE_A="${INITIAL_POSE_A:--3.119}"

PIDS=()
CLEANED_UP=0

cleanup() {
  if [[ "${CLEANED_UP}" == "1" ]]; then
    return
  fi
  CLEANED_UP=1
  echo
  echo "[start_navigation] Stopping launched processes..."
  for pid in "${PIDS[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
  wait 2>/dev/null || true
}

start_process() {
  local name="$1"
  shift
  echo "[start_navigation] Starting ${name}: $*"
  "$@" &
  PIDS+=("$!")
}

if [[ ! -f "${SETUP_FILE}" ]]; then
  echo "[start_navigation] Missing ROS setup file: ${SETUP_FILE}" >&2
  echo "[start_navigation] Build first: cd ${WORKSPACE} && colcon build --symlink-install" >&2
  exit 1
fi

source "${SETUP_FILE}"

# FastDDS 参数：避免共享内存端口锁失败，例如 "Failed init_port fastrtps_port..."。
export ROS_DISABLE_LOANED_MESSAGES=1
export RMW_FASTRTPS_USE_QOS_FROM_XML=0
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
unset FASTRTPS_DEFAULT_PROFILES_FILE

trap cleanup EXIT INT TERM

if [[ -f "${RVIZ_CONFIG_FILE}" ]]; then
  start_process "RViz2" rviz2 -d "${RVIZ_CONFIG_FILE}" --ros-args -p use_sim_time:=False
else
  start_process "RViz2" rviz2 --ros-args -p use_sim_time:=False
fi
sleep "${RVIZ_DELAY}"

start_process "base bringup" ros2 launch origincar_base origincar_bringup.launch.xml \
  start_camera:="${START_CAMERA}"
sleep "${BRINGUP_DELAY}"

if [[ "${START_QR_DETECTOR}" == "1" ]]; then
  if [[ "${START_CAMERA}" != "1" ]]; then
    echo "[start_navigation] Warning: QR detector needs USB camera image topic, but START_CAMERA is not 1."
  fi
  start_process "QR detector" ros2 run qr_detector qr_detector_node --ros-args \
    -p image_topic:="${QR_IMAGE_TOPIC}" \
    -p qr_topic:="${QR_TOPIC}" \
    -p scan_hz:="${QR_SCAN_HZ}" \
    -p detector_engine:="${QR_DETECTOR_ENGINE}" \
    -p resize_width:="${QR_RESIZE_WIDTH}" \
    -p repeat_publish_interval:="${QR_REPEAT_PUBLISH_INTERVAL}"
  sleep "${QR_DELAY}"
fi

if [[ "${ENABLE_QR_DIRECTION_SELECT}" == "true" && "${START_QR_DETECTOR}" != "1" ]]; then
  echo "[start_navigation] Warning: QR route direction select is enabled, but START_QR_DETECTOR is not 1."
fi

if [[ "${START_IMAGE_ANALYZER}" == "1" && "${START_CAMERA}" != "1" ]]; then
  echo "[start_navigation] Warning: image analyzer needs USB camera image topic, but START_CAMERA is not 1."
fi

if [[ "${START_FULL_NAV2}" == "1" ]]; then
  start_process "Nav2 AMCL keepout" ros2 launch origincar_system nav2_amcl_keepout.launch.xml \
    map:="${MAP_FILE}" \
    initial_pose_x:="${INITIAL_POSE_X}" \
    initial_pose_y:="${INITIAL_POSE_Y}" \
    initial_pose_a:="${INITIAL_POSE_A}"
else
  start_process "map and AMCL localization" ros2 launch origincar_system map_only.launch.xml \
    map:="${MAP_FILE}" \
    keepout_map:="${KEEPOUT_MAP_FILE}"
fi
sleep "${NAV2_DELAY}"

if [[ "${START_MULTI_POINT}" == "1" ]]; then
  if [[ "${WAIT_FOR_NAV_START}" == "1" ]]; then
    echo
    echo "[start_navigation] RViz2, base bringup, and Nav2 are running."
    echo "[start_navigation] Set the initial pose in RViz2 first."
    read -r -p "[start_navigation] Press Enter here to start multi-point navigation..." _ || true
  fi

  if [[ "${NAV_MODE}" == "nav2_goals" ]]; then
    start_process "multi-point navigation" ros2 launch origincar_system multi_point_nav.launch.xml \
      waypoints_file:="${WAYPOINTS_FILE}" \
      action_type:=ordered_smooth \
      pass_radius:="${PASS_RADIUS}" \
      max_goal_retries:="${MAX_GOAL_RETRIES}" \
      route_direction:="${ROUTE_DIRECTION}" \
      enable_qr_direction_select:="${ENABLE_QR_DIRECTION_SELECT}" \
      qr_topic:="${QR_TOPIC}" \
      use_qr_parity_direction:="${USE_QR_PARITY_DIRECTION}" \
      clockwise_qr_value:="${CLOCKWISE_QR_VALUE}" \
      counterclockwise_qr_value:="${COUNTERCLOCKWISE_QR_VALUE}" \
      start_image_analyzer:="${START_IMAGE_ANALYZER}" \
      image_analysis_waypoint:="${IMAGE_ANALYSIS_WAYPOINT}" \
      image_analysis_trigger_topic:="${IMAGE_ANALYSIS_TRIGGER_TOPIC}" \
      image_topic:="${IMAGE_ANALYSIS_IMAGE_TOPIC}" \
      image_msg_type:="${IMAGE_ANALYSIS_IMAGE_MSG_TYPE}" \
      image_analysis_result_topic:="${IMAGE_ANALYSIS_RESULT_TOPIC}"
  else
    start_process "smooth path follower" ros2 launch origincar_system smooth_path_follower.launch.xml \
      waypoints_file:="${WAYPOINTS_FILE}" \
      pass_radius:="${PASS_RADIUS}" \
      lookahead_distance:="${LOOKAHEAD_DISTANCE}" \
      enable_smooth_keepout_path:="${ENABLE_SMOOTH_KEEPOUT_PATH}" \
      smooth_path_min_waypoint:="${SMOOTH_PATH_MIN_WAYPOINT}" \
      smooth_path_max_waypoint:="${SMOOTH_PATH_MAX_WAYPOINT}" \
      smooth_path_samples_per_segment:="${SMOOTH_PATH_SAMPLES_PER_SEGMENT}" \
      smooth_path_keepout_clearance:="${SMOOTH_PATH_KEEPOUT_CLEARANCE}" \
      smooth_path_keepout_weight:="${SMOOTH_PATH_KEEPOUT_WEIGHT}" \
      enable_skip_overshot_waypoints:="${ENABLE_SKIP_OVERSHOT_WAYPOINTS}" \
      overshot_waypoint_margin:="${OVERSHOT_WAYPOINT_MARGIN}" \
      overshot_waypoint_max_distance:="${OVERSHOT_WAYPOINT_MAX_DISTANCE}" \
      overshot_next_waypoint_max_distance:="${OVERSHOT_NEXT_WAYPOINT_MAX_DISTANCE}" \
      skip_overshot_min_waypoint:="${SKIP_OVERSHOT_MIN_WAYPOINT}" \
      skip_overshot_max_waypoint:="${SKIP_OVERSHOT_MAX_WAYPOINT}" \
      linear_speed:="${LINEAR_SPEED}" \
      channel_linear_speed:="${CHANNEL_LINEAR_SPEED}" \
      channel_waypoint_ranges:="${CHANNEL_WAYPOINT_RANGES}" \
      max_angular_z:="${MAX_ANGULAR_Z}" \
      turn_angular_gain:="${TURN_ANGULAR_GAIN}" \
      turn_min_speed_scale:="${TURN_MIN_SPEED_SCALE}" \
      reverse_angular_gain:="${REVERSE_ANGULAR_GAIN}" \
      reverse_min_speed_scale:="${REVERSE_MIN_SPEED_SCALE}" \
      reverse_goal_lookahead_distance:="${REVERSE_GOAL_LOOKAHEAD_DISTANCE}" \
      reverse_pass_radius:="${REVERSE_PASS_RADIUS}" \
      enable_reverse_approach_nudge:="${ENABLE_REVERSE_APPROACH_NUDGE}" \
      reverse_approach_nudge_waypoint:="${REVERSE_APPROACH_NUDGE_WAYPOINT}" \
      reverse_approach_nudge_distance:="${REVERSE_APPROACH_NUDGE_DISTANCE}" \
      reverse_approach_nudge_angular_z:="${REVERSE_APPROACH_NUDGE_ANGULAR_Z}" \
      obstacle_stop_distance:="${OBSTACLE_STOP_DISTANCE}" \
      obstacle_enable_from_waypoint:="${OBSTACLE_ENABLE_FROM_WAYPOINT}" \
      obstacle_slow_distance:="${OBSTACLE_SLOW_DISTANCE}" \
      obstacle_avoid_distance:="${OBSTACLE_AVOID_DISTANCE}" \
      front_angle_deg:="${FRONT_ANGLE_DEG}" \
      enable_keepout_avoidance:="${ENABLE_KEEPOUT_AVOIDANCE}" \
      keepout_mask_topic:="${KEEPOUT_MASK_TOPIC}" \
      keepout_occupied_threshold:="${KEEPOUT_OCCUPIED_THRESHOLD}" \
      keepout_stop_distance:="${KEEPOUT_STOP_DISTANCE}" \
      keepout_slow_distance:="${KEEPOUT_SLOW_DISTANCE}" \
      keepout_avoid_distance:="${KEEPOUT_AVOID_DISTANCE}" \
      keepout_side_sample_distance:="${KEEPOUT_SIDE_SAMPLE_DISTANCE}" \
      enable_keepout_hard_stop:="${ENABLE_KEEPOUT_HARD_STOP}" \
      keepout_min_speed_scale:="${KEEPOUT_MIN_SPEED_SCALE}" \
      backup_trigger_time:="${BACKUP_TRIGGER_TIME}" \
      backup_duration:="${BACKUP_DURATION}" \
      backup_speed:="${BACKUP_SPEED}" \
      backup_angular_z:="${BACKUP_ANGULAR_Z}" \
      backup_stop_distance:="${BACKUP_STOP_DISTANCE}" \
      enable_qr_skip_first:="${ENABLE_QR_SKIP_FIRST}" \
      qr_topic:="${QR_TOPIC}" \
      route_direction:="${ROUTE_DIRECTION}" \
      enable_qr_direction_select:="${ENABLE_QR_DIRECTION_SELECT}" \
      use_qr_parity_direction:="${USE_QR_PARITY_DIRECTION}" \
      clockwise_qr_value:="${CLOCKWISE_QR_VALUE}" \
      counterclockwise_qr_value:="${COUNTERCLOCKWISE_QR_VALUE}" \
      start_image_analyzer:="${START_IMAGE_ANALYZER}" \
      image_analysis_waypoint:="${IMAGE_ANALYSIS_WAYPOINT}" \
      image_analysis_trigger_topic:="${IMAGE_ANALYSIS_TRIGGER_TOPIC}" \
      image_topic:="${IMAGE_ANALYSIS_IMAGE_TOPIC}" \
      image_msg_type:="${IMAGE_ANALYSIS_IMAGE_MSG_TYPE}" \
      image_analysis_result_topic:="${IMAGE_ANALYSIS_RESULT_TOPIC}"
  fi
else
  echo
  echo "[start_navigation] RViz2, base bringup, and Nav2 are running."
  echo "[start_navigation] Multi-point navigation is disabled; the car will not start moving."
  echo "[start_navigation] To start patrol, run with START_MULTI_POINT=1."
fi

echo "[start_navigation] All requested processes are running."
echo "[start_navigation] Press Ctrl+C in this terminal to stop them."
wait || true
