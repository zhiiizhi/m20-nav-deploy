#!/bin/bash
# ============================================================
# 一键停止/恢复 106 导航主机上的原厂导航服务
# ============================================================
# 用途：避免原厂导航和你的 lightning 定位 + /NAV_CMD 控制冲突
#
# 用法（在 104 上，先 su 提权）：
#   su                          # 密码: '
#   cd /var/opt/robot/data/sdk_deploy
#   ./stop_native_nav.sh          # 停止原厂导航服务
#   ./stop_native_nav.sh status   # 查看当前状态
#   ./stop_native_nav.sh start    # 恢复原厂导航服务
#
# 注意：重启机器狗后原厂服务会自动恢复，需重新运行 stop
# ============================================================

ROBOT_NAV_HOST="10.21.31.106"

# 原厂导航服务（会和 lightning 定位 + /NAV_CMD 冲突）
# handler 是总调度会自动拉起其他服务，停止时先停它
SERVICES="localization global_planner planner passable_area handler"
SERVICES_STOP_ORDER="handler localization global_planner planner passable_area"

# 颜色
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

# SSH 密码（单引号）
REMOTE_PASS="'"

# 远程执行（已配好 SSH_ASKPASS 免密登录 106）
# 注意：systemctl is-active 不需要 sudo，只有 stop/start 需要
remote_exec() {
    local cmd="$1"
    setsid ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
        user@$ROBOT_NAV_HOST "$cmd" 2>&1 | grep -v "Warning: Permanently"
}

# 远程停止单个服务
# 密码是单引号 '（ASCII 0x27），直接写在命令行会破坏 shell 引号，
# 所以用 printf '\47' 在远程生成单引号字符作为 sudo 密码
remote_stop_service() {
    local svc="$1"
    setsid ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
        user@$ROBOT_NAV_HOST "printf '\47' | sudo -S systemctl stop $svc.service" \
        2>&1 | grep -v "Warning: Permanently" | grep -v "sudo\]" | grep -v "^$"
}

# 远程启动单个服务
remote_start_service() {
    local svc="$1"
    setsid ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
        user@$ROBOT_NAV_HOST "printf '\47' | sudo -S systemctl start $svc.service" \
        2>&1 | grep -v "Warning: Permanently" | grep -v "sudo\]" | grep -v "^$"
}

# 检查是否 root（需要 root 权限设置 askpass）
check_root() {
    if [ "$(id -u)" != "0" ]; then
        echo -e "${RED}[错误] 请先 su 提权后再运行此脚本${NC}"
        echo "  su        # 密码: '"
        echo "  ./stop_native_nav.sh"
        exit 1
    fi
}

# 设置 SSH 免密（askpass 方式连 106）
setup_ssh_askpass() {
    local askpass="/tmp/askpass_stop_nav.sh"
    printf '#!/bin/bash\necho "%s"\n' "$REMOTE_PASS" > "$askpass"
    chmod +x "$askpass"
    export SSH_ASKPASS="$askpass"
    export SSH_ASKPASS_REQUIRE=force
    export DISPLAY=:0
}

# 检查连通性
check_connect() {
    if ! ping -c 1 -W 2 $ROBOT_NAV_HOST >/dev/null 2>&1; then
        echo -e "${RED}[错误] 无法连接 $ROBOT_NAV_HOST${NC}"
        exit 1
    fi
}

# 显示服务状态
show_status() {
    check_connect
    echo -e "${YELLOW}========== 106 导航主机服务状态 ==========${NC}"
    echo ""
    # 直接硬编码远程命令，避免变量拼接的转义问题
    result=$(remote_exec "for s in localization global_planner planner passable_area handler; do echo \$s \$(systemctl is-active \$s.service 2>/dev/null); done")
    echo "$result" | while read svc state; do
        if [ "$state" = "active" ]; then
            printf "  ${GREEN}%-24s 运行中${NC}\n" "$svc.service"
        else
            printf "  ${RED}%-24s 已停止${NC}\n" "$svc.service"
        fi
    done
    echo ""
    echo "提示：运行中的服务会和 lightning 定位 + /NAV_CMD 冲突"
}

# 停止服务（先停 handler，避免它拉起其他服务）
stop_services() {
    check_connect
    echo -e "${YELLOW}========== 停止 106 原厂导航服务 ==========${NC}"
    echo ""
    # 先停 handler（总调度），等2秒，再逐个停其他
    remote_stop_service "handler"
    sleep 2
    for svc in localization global_planner planner passable_area; do
        remote_stop_service "$svc"
    done
    sleep 1
    # 显示结果（is-active 不需 sudo）
    result=$(remote_exec "for s in localization global_planner planner passable_area handler; do echo \$s \$(systemctl is-active \$s.service 2>/dev/null); done")
    echo "$result" | while read svc state; do
        if [ "$state" = "active" ]; then
            printf "  ${RED}%-24s 停止失败${NC}\n" "$svc.service"
        else
            printf "  ${GREEN}%-24s 已停止${NC}\n" "$svc.service"
        fi
    done
    echo ""
    echo "现在可以：1.运行 lightning 定位  2.运行 /NAV_CMD 键盘遥控"
}

# 恢复服务
start_services() {
    check_connect
    echo -e "${YELLOW}========== 恢复 106 原厂导航服务 ==========${NC}"
    echo ""
    # 先启动其他，最后启动 handler
    for svc in localization global_planner planner passable_area; do
        remote_start_service "$svc"
    done
    sleep 1
    remote_start_service "handler"
    sleep 1
    result=$(remote_exec "for s in localization global_planner planner passable_area handler; do echo \$s \$(systemctl is-active \$s.service 2>/dev/null); done")
    echo "$result" | while read svc state; do
        if [ "$state" = "active" ]; then
            printf "  ${GREEN}%-24s 已启动${NC}\n" "$svc.service"
        else
            printf "  ${RED}%-24s 启动失败${NC}\n" "$svc.service"
        fi
    done
}

# === 主逻辑 ===
check_root
setup_ssh_askpass

case "${1:-stop}" in
    stop)   stop_services ;;
    start)  start_services ;;
    status) show_status ;;
    *)
        echo "用法: $0 [stop|start|status]"
        echo "  stop   - 停止原厂导航服务（默认）"
        echo "  start  - 恢复原厂导航服务"
        echo "  status - 查看当前状态"
        echo ""
        echo "请先 su 提权：su  （密码: '）"
        exit 1
        ;;
esac
