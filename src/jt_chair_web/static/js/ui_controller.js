/**
 * ui_controller.js - 主控逻辑
 * WebSocket 通信、按钮事件、模式切换
 */

// ==========================================
// WebSocket 连接
// ==========================================
const socket = io();

// --- 接收状态更新 ---
socket.on('state_update', (data) => {
    // 更新地图
    if (data.map_image && data.map_info) {
        MapRenderer.updateMap(data.map_image, data.map_info);
    }

    // 更新机器人位姿
    if (data.robot_pose) {
        MapRenderer.updateRobotPose(data.robot_pose);
    }

    // 更新路径
    if (data.global_path) {
        MapRenderer.updatePath(data.global_path);
    }

    // 更新状态指示
    updateStatusBar(data);
});

socket.on('connect', () => {
    console.log('WebSocket 已连接');
});

socket.on('disconnect', () => {
    console.log('WebSocket 已断开');
    showToast('连接已断开，正在重连...', 'error');
});

// ==========================================
// 状态栏更新
// ==========================================
function updateStatusBar(data) {
    // 导航状态
    const navDot = document.querySelector('#status-nav2 .status-dot');
    if (data.nav2_running) {
        navDot.className = 'status-dot on';
    } else {
        navDot.className = 'status-dot off';
    }

    // 手柄状态
    const cmdDot = document.querySelector('#status-cmdvel .status-dot');
    if (data.cmdvel_running) {
        cmdDot.className = 'status-dot on';
    } else {
        cmdDot.className = 'status-dot off';
    }

    // 模式文本
    const modeNames = ['禁止横移', '禁止旋转', '仅旋转'];
    document.getElementById('mode-text').textContent = modeNames[data.mode] || '未知';

    // 速度文本
    const speedNames = ['低速', '中速', '高速'];
    document.getElementById('speed-text').textContent = speedNames[data.speed_level] || '中速';

    // 更新模式按钮高亮
    document.querySelectorAll('.btn-mode').forEach((btn, i) => {
        btn.classList.toggle('active', i === data.mode);
    });

    // 更新速度按钮高亮
    document.querySelectorAll('.btn-speed').forEach((btn, i) => {
        btn.classList.toggle('active', i === data.speed_level);
    });

    // 导航按钮状态
    const btnStart = document.getElementById('btn-nav-start');
    const btnStop = document.getElementById('btn-nav-stop');
    if (data.nav2_running) {
        btnStart.style.display = 'none';
        btnStop.style.display = '';
    } else {
        btnStart.style.display = '';
        btnStop.style.display = 'none';
    }

    // 手柄按钮文字
    const btnCmd = document.getElementById('btn-cmdvel');
    if (data.cmdvel_running) {
        btnCmd.querySelector('.btn-label').textContent = '关闭手柄';
    } else {
        btnCmd.querySelector('.btn-label').textContent = '启用手柄';
    }
}

// ==========================================
// API 调用辅助
// ==========================================
async function apiCall(url, method = 'GET', body = null) {
    try {
        const opts = { method };
        if (body) {
            opts.headers = { 'Content-Type': 'application/json' };
            opts.body = JSON.stringify(body);
        }
        const resp = await fetch(url, opts);
        return await resp.json();
    } catch (e) {
        console.error('API 错误:', e);
        return { success: false, message: '网络请求失败' };
    }
}

// ==========================================
// Toast 提示
// ==========================================
let toastTimer = null;
function showToast(msg, type = 'success') {
    const toast = document.getElementById('toast');
    toast.textContent = msg;
    toast.className = `toast ${type}`;
    if (toastTimer) clearTimeout(toastTimer);
    toastTimer = setTimeout(() => {
        toast.className = 'toast hidden';
    }, 2500);
}

// ==========================================
// 导航启停
// ==========================================
async function toggleNav2() {
    const btnStart = document.getElementById('btn-nav-start');
    const isRunning = btnStart.style.display === 'none';

    if (isRunning) {
        showToast('正在关闭导航程序...');
        const result = await apiCall('/api/nav2/stop', 'POST');
        showToast(result.message, result.success ? 'success' : 'error');
    } else {
        showToast('正在启动导航程序...');
        const result = await apiCall('/api/nav2/start', 'POST');
        showToast(result.message, result.success ? 'success' : 'error');
    }
}

// ==========================================
// 手柄控制启停
// ==========================================
async function toggleCmdVel() {
    const btnCmd = document.getElementById('btn-cmdvel');
    const label = btnCmd.querySelector('.btn-label').textContent;
    const isRunning = label === '关闭手柄';

    if (isRunning) {
        showToast('正在关闭手柄控制...');
        const result = await apiCall('/api/cmdvel/stop', 'POST');
        showToast(result.message, result.success ? 'success' : 'error');
    } else {
        showToast('正在启用手柄控制...');
        const result = await apiCall('/api/cmdvel/start', 'POST');
        showToast(result.message, result.success ? 'success' : 'error');
    }
}

// ==========================================
// 模式设置
// ==========================================
let currentMode = 0;
async function setMode(mode) {
    currentMode = mode;
    const speedLevel = currentSpeedLevel;  // 保持当前速度档位
    const result = await apiCall('/api/profile', 'POST', {
        mode: mode,
        speed_level: speedLevel
    });
    if (result.success) {
        document.querySelectorAll('.btn-mode').forEach((btn, i) => {
            btn.classList.toggle('active', i === mode);
        });
    }
}

// ==========================================
// 速度设置
// ==========================================
let currentSpeedLevel = 1;
async function setSpeed(level) {
    currentSpeedLevel = level;
    const result = await apiCall('/api/profile', 'POST', {
        mode: currentMode,
        speed_level: level
    });
    if (result.success) {
        document.querySelectorAll('.btn-speed').forEach((btn, i) => {
            btn.classList.toggle('active', i === level);
        });
        const speedNames = ['低速', '中速', '高速'];
        document.getElementById('speed-text').textContent = speedNames[level];
    }
}

// ==========================================
// 设置初始位姿模式
// ==========================================
function enableSetPose() {
    if (MapRenderer.mode === 'set_pose') {
        exitPlaceMode();
        showToast('已取消初始位姿模式');
        return;
    }

    MapRenderer.mode = 'set_pose';
    MapRenderer.posePlacing = null;
    document.getElementById('btn-set-pose').style.border = '3px solid #e94560';
    document.getElementById('btn-set-goal').style.border = '';

    const overlay = document.getElementById('mode-overlay');
    overlay.classList.remove('hidden');
    document.getElementById('mode-overlay-text').textContent = '📍 按住拖拽设置初始位姿和方向';

    showToast('在地图上按住并拖拽来设置位姿和方向');
}

// ==========================================
// 设置导航目标点模式
// ==========================================
function enableSetGoal() {
    if (MapRenderer.mode === 'set_goal') {
        exitPlaceMode();
        showToast('已取消目标点模式');
        return;
    }

    MapRenderer.mode = 'set_goal';
    MapRenderer.posePlacing = null;
    document.getElementById('btn-set-goal').style.border = '3px solid #e94560';
    document.getElementById('btn-set-pose').style.border = '';

    const overlay = document.getElementById('mode-overlay');
    overlay.classList.remove('hidden');
    document.getElementById('mode-overlay-text').textContent = '🎯 按住拖拽设置目标点和方向';

    showToast('在地图上按住并拖拽来设置目标点和方向');
}

function exitPlaceMode() {
    MapRenderer.mode = 'view';
    MapRenderer.posePlacing = null;
    document.getElementById('btn-set-pose').style.border = '';
    document.getElementById('btn-set-goal').style.border = '';
    document.getElementById('mode-overlay').classList.add('hidden');
}

// ==========================================
// 设置初始位姿回调（由 MapRenderer 调用）
// ==========================================
MapRenderer.setPoseCallback = async function(mx, my, yaw) {
    const result = await apiCall('/api/initialpose', 'POST', { x: mx, y: my, yaw: yaw });

    if (result.success) {
        const yawDeg = (yaw * 180 / Math.PI).toFixed(1);
        showToast(`初始位姿已设置: (${mx.toFixed(2)}, ${my.toFixed(2)}, ${yawDeg}°)`, 'success');
    } else {
        showToast('初始位姿设置失败', 'error');
    }

    exitPlaceMode();
};

// ==========================================
// 设置导航目标回调（由 MapRenderer 调用）
// ==========================================
MapRenderer.setGoalCallback = async function(mx, my, yaw) {
    const result = await apiCall('/api/navgoal', 'POST', { x: mx, y: my, yaw: yaw });

    if (result.success) {
        const yawDeg = (yaw * 180 / Math.PI).toFixed(1);
        showToast(`导航目标已发送: (${mx.toFixed(2)}, ${my.toFixed(2)}, ${yawDeg}°)`, 'success');
    } else {
        showToast('导航目标发送失败，请确认导航系统已启动', 'error');
    }

    exitPlaceMode();
};

// 监听地图模式退出事件（拖拽释放后自动退出）
window.addEventListener('map_mode_exit', () => {
    exitPlaceMode();
});

// ==========================================
// 记录房间
// ==========================================
function toggleRoomMenu() {
    const modal = document.getElementById('room-modal');
    modal.classList.toggle('hidden');
}

async function recordRoom(roomKey) {
    toggleRoomMenu();  // 关闭弹窗

    const roomNames = {
        'living_room': '客厅',
        'kitchen': '厨房',
        'master_bedroom': '主卧',
        'guest_bedroom': '客卧',
        'toilet': '厕所',
        'toilet_dock': '马桶',
    };

    showToast(`正在记录${roomNames[roomKey] || roomKey}坐标...`);
    const result = await apiCall('/api/record_room', 'POST', { room: roomKey });
    showToast(result.message, result.success ? 'success' : 'error');
}

// ==========================================
// 座椅控制
// ==========================================
async function controlSeat(motorId, percent) {
    const action = percent > 0 ? '上升' : percent < 0 ? '下降' : '停止';
    const result = await apiCall('/api/seat', 'POST', {
        motor_id: motorId,
        percent: percent
    });
    // 座椅控制静默处理，不弹 toast（避免频繁提示）
    if (!result.success) {
        showToast('座椅控制失败', 'error');
    }
}

// ==========================================
// 初始化
// ==========================================
document.addEventListener('DOMContentLoaded', () => {
    MapRenderer.init('map-canvas');

    // 定期获取状态（WebSocket 断开时的后备方案）
    setInterval(async () => {
        if (!socket.connected) {
            try {
                const resp = await fetch('/api/status');
                const data = await resp.json();
                updateStatusBar(data);
            } catch (e) { /* 忽略 */ }
        }
    }, 5000);

    // 初始状态查询
    fetch('/api/status')
        .then(r => r.json())
        .then(data => updateStatusBar(data))
        .catch(() => {});

    console.log('🏥 智能轮椅控制台已就绪');
});
