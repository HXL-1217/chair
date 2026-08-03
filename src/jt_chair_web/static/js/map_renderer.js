/**
 * map_renderer.js - 地图渲染引擎
 * 负责 Canvas 地图绘制、轮椅位姿、导航路径
 */

const MapRenderer = {
    // --- 状态 ---
    canvas: null,
    ctx: null,
    mapImage: null,         // 地图 PNG Image 对象
    mapInfo: null,          // {width, height, resolution, origin_x, origin_y, origin_yaw}
    robotPose: null,        // {x, y, yaw} 米制坐标
    globalPath: [],         // [{x, y}, ...] 米制坐标

    // --- 视图变换 ---
    viewX: 0,              // 画布中心对应的地图 X（米）
    viewY: 0,              // 画布中心对应的地图 Y（米）
    viewScale: 30,         // 像素/米（默认 1m = 30px）
    targetScale: 30,       // 目标缩放（用于平滑过渡）
    targetViewX: 0,        // 目标中心 X
    targetViewY: 0,        // 目标中心 Y

    // --- 交互状态 ---
    mode: 'view',          // 'view' | 'set_pose' | 'set_goal'
    setPoseCallback: null,
    setGoalCallback: null,
    coordCallback: null,   // 鼠标移动时的坐标回调

    // --- 拖拽 ---
    isDragging: false,
    dragStartX: 0,
    dragStartY: 0,
    dragStartViewX: 0,
    dragStartViewY: 0,

    // --- 位姿/目标拖拽放置 ---
    posePlacing: null,     // {startMapX, startMapY, curMapX, curMapY} 当前正在拖拽放置的位姿
    poseCursorX: 0,        // 当前鼠标/触摸在画布上的 X
    poseCursorY: 0,        // 当前鼠标/触摸在画布上的 Y

    // --- 缩放动画 ---
    lastWheelTime: 0,

    // --- 初始化 ---
    init(canvasId) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext('2d');

        // 适配高 DPI
        this.resize();
        window.addEventListener('resize', () => this.resize());

        // 触摸事件
        this.canvas.addEventListener('touchstart', (e) => this.onTouchStart(e), {passive: false});
        this.canvas.addEventListener('touchmove', (e) => this.onTouchMove(e), {passive: false});
        this.canvas.addEventListener('touchend', (e) => this.onTouchEnd(e));

        // 鼠标事件
        this.canvas.addEventListener('mousedown', (e) => this.onMouseDown(e));
        this.canvas.addEventListener('mousemove', (e) => this.onMouseMove(e));
        this.canvas.addEventListener('mouseup', (e) => this.onMouseUp(e));
        this.canvas.addEventListener('mouseleave', (e) => this.onMouseUp(e));

        // 滚轮缩放
        this.canvas.addEventListener('wheel', (e) => this.onWheel(e), {passive: false});

        // 双指缩放
        this.pinchStartDist = 0;
        this.pinchStartScale = 0;

        // 初始动画帧
        this.renderLoop();
    },

    // --- 画布尺寸适配 ---
    resize() {
        const rect = this.canvas.parentElement.getBoundingClientRect();
        const dpr = window.devicePixelRatio || 1;
        this.canvas.width = rect.width * dpr;
        this.canvas.height = rect.height * dpr;
        this.canvas.style.width = rect.width + 'px';
        this.canvas.style.height = rect.height + 'px';
        this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        this.width = rect.width;
        this.height = rect.height;
    },

    // --- 数据更新接口 ---
    updateMap(mapBase64, mapInfo) {
        if (!mapBase64) return;
        const isNewMap = !this.mapInfo || 
            this.mapInfo.width !== mapInfo.width || 
            this.mapInfo.height !== mapInfo.height;
        
        this.mapInfo = mapInfo;

        const img = new Image();
        img.onload = () => {
            this.mapImage = img;
            // 首次加载地图或地图尺寸变化时才重新居中
            if (isNewMap) {
                const cx = mapInfo.origin_x + mapInfo.width * mapInfo.resolution / 2;
                const cy = mapInfo.origin_y + mapInfo.height * mapInfo.resolution / 2;
                this.viewX = this.targetViewX = cx;
                this.viewY = this.targetViewY = cy;
                // 自适应缩放：让地图填满画布
                const mapW = mapInfo.width * mapInfo.resolution;
                const mapH = mapInfo.height * mapInfo.resolution;
                const scaleX = (this.width * 0.85) / mapW;
                const scaleY = (this.height * 0.85) / mapH;
                this.viewScale = this.targetScale = Math.min(scaleX, scaleY, 80);
            }
        };
        img.src = 'data:image/png;base64,' + mapBase64;
    },

    updateRobotPose(pose) {
        this.robotPose = pose;
    },

    updatePath(path) {
        this.globalPath = path || [];
    },

    // --- 坐标转换 ---
    /** 地图坐标(米) → 画布坐标(像素) */
    mapToCanvas(mx, my) {
        const cx = (mx - this.viewX) * this.viewScale + this.width / 2;
        const cy = (my - this.viewY) * this.viewScale + this.height / 2;
        return { x: cx, y: cy };
    },

    /** 画布坐标(像素) → 地图坐标(米) */
    canvasToMap(cx, cy) {
        const mx = (cx - this.width / 2) / this.viewScale + this.viewX;
        const my = (cy - this.height / 2) / this.viewScale + this.viewY;
        return { x: mx, y: my };
    },

    // --- 渲染循环（含平滑缩放） ---
    renderLoop() {
        // 平滑过渡到目标缩放
        const lerp = 0.25;  // 平滑系数（越小越平滑）
        if (Math.abs(this.targetScale - this.viewScale) > 0.05) {
            this.viewScale += (this.targetScale - this.viewScale) * lerp;
        } else {
            this.viewScale = this.targetScale;
        }
        if (Math.abs(this.targetViewX - this.viewX) > 0.001 || 
            Math.abs(this.targetViewY - this.viewY) > 0.001) {
            this.viewX += (this.targetViewX - this.viewX) * lerp;
            this.viewY += (this.targetViewY - this.viewY) * lerp;
        } else {
            this.viewX = this.targetViewX;
            this.viewY = this.targetViewY;
        }

        this.render();
        requestAnimationFrame(() => this.renderLoop());
    },

    render() {
        const ctx = this.ctx;
        const w = this.width;
        const h = this.height;

        // 清空
        ctx.clearRect(0, 0, w, h);

        // 背景
        ctx.fillStyle = '#0a0a1a';
        ctx.fillRect(0, 0, w, h);

        // --- 绘制地图 ---
        if (this.mapImage && this.mapInfo) {
            const info = this.mapInfo;
            // 地图左上角在画布中的位置
            const origin = this.mapToCanvas(info.origin_x, info.origin_y);
            // 地图宽高（像素）
            const mapW = info.width * info.resolution * this.viewScale;
            const mapH = info.height * info.resolution * this.viewScale;

            ctx.save();
            // 如果地图有旋转
            if (info.origin_yaw && Math.abs(info.origin_yaw) > 0.001) {
                const oc = this.mapToCanvas(info.origin_x, info.origin_y);
                ctx.translate(oc.x, oc.y);
                ctx.rotate(-info.origin_yaw);
                ctx.drawImage(this.mapImage, 0, 0, mapW, mapH);
            } else {
                ctx.drawImage(this.mapImage, origin.x, origin.y, mapW, mapH);
            }
            ctx.restore();
        }

        // --- 绘制全局路径 ---
        if (this.globalPath.length > 1) {
            ctx.beginPath();
            ctx.strokeStyle = '#00d2ff';
            ctx.lineWidth = 2.5;
            ctx.lineCap = 'round';
            ctx.lineJoin = 'round';
            ctx.setLineDash([]);
            ctx.shadowColor = 'rgba(0,210,255,0.5)';
            ctx.shadowBlur = 6;

            const first = this.mapToCanvas(this.globalPath[0].x, this.globalPath[0].y);
            ctx.moveTo(first.x, first.y);
            for (let i = 1; i < this.globalPath.length; i++) {
                const pt = this.mapToCanvas(this.globalPath[i].x, this.globalPath[i].y);
                ctx.lineTo(pt.x, pt.y);
            }
            ctx.stroke();
            ctx.shadowBlur = 0;
        }

        // --- 绘制路径起终点标记 ---
        if (this.globalPath.length > 0) {
            const startPt = this.mapToCanvas(this.globalPath[0].x, this.globalPath[0].y);
            ctx.fillStyle = '#00d2ff';
            ctx.beginPath();
            ctx.arc(startPt.x, startPt.y, 5, 0, Math.PI * 2);
            ctx.fill();

            const endPt = this.mapToCanvas(
                this.globalPath[this.globalPath.length - 1].x,
                this.globalPath[this.globalPath.length - 1].y
            );
            ctx.fillStyle = '#e94560';
            ctx.beginPath();
            ctx.arc(endPt.x, endPt.y, 6, 0, Math.PI * 2);
            ctx.fill();
        }

        // --- 绘制机器人位姿 ---
        if (this.robotPose) {
            const pos = this.mapToCanvas(this.robotPose.x, this.robotPose.y);
            const yaw = this.robotPose.yaw || 0;

            // 三角形箭头
            const size = 14;
            const tipX = pos.x + Math.cos(yaw) * size * 1.2;
            const tipY = pos.y + Math.sin(yaw) * size * 1.2;
            const leftX = pos.x + Math.cos(yaw + 2.5) * size * 0.7;
            const leftY = pos.y + Math.sin(yaw + 2.5) * size * 0.7;
            const rightX = pos.x + Math.cos(yaw - 2.5) * size * 0.7;
            const rightY = pos.y + Math.sin(yaw - 2.5) * size * 0.7;

            ctx.save();
            // 发光效果
            ctx.shadowColor = 'rgba(46,204,113,0.6)';
            ctx.shadowBlur = 10;

            ctx.beginPath();
            ctx.moveTo(tipX, tipY);
            ctx.lineTo(leftX, leftY);
            ctx.lineTo(rightX, rightY);
            ctx.closePath();
            ctx.fillStyle = '#2ecc71';
            ctx.fill();
            ctx.strokeStyle = '#27ae60';
            ctx.lineWidth = 2;
            ctx.stroke();

            ctx.restore();

            // 中心小圆
            ctx.beginPath();
            ctx.arc(pos.x, pos.y, 4, 0, Math.PI * 2);
            ctx.fillStyle = '#ffffff';
            ctx.fill();
        }

        // --- 绘制位姿/目标放置预览（拖拽中） ---
        if (this.posePlacing) {
            const sx = this.posePlacing.startMapX;
            const sy = this.posePlacing.startMapY;
            const ex = this.posePlacing.curMapX;
            const ey = this.posePlacing.curMapY;
            const start = this.mapToCanvas(sx, sy);
            const end = this.mapToCanvas(ex, ey);
            const dragDist = Math.sqrt((end.x - start.x) ** 2 + (end.y - start.y) ** 2);

            ctx.save();
            // 半透明圆形标记起点
            ctx.fillStyle = this.mode === 'set_pose' ? 'rgba(108, 92, 231, 0.5)' : 'rgba(233, 69, 96, 0.5)';
            ctx.strokeStyle = this.mode === 'set_pose' ? '#6c5ce7' : '#e94560';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.arc(start.x, start.y, 8, 0, Math.PI * 2);
            ctx.fill();
            ctx.stroke();

            // 方向线（从起点到当前光标）
            ctx.strokeStyle = this.mode === 'set_pose' ? '#6c5ce7' : '#e94560';
            ctx.lineWidth = 2;
            ctx.setLineDash([6, 4]);
            ctx.beginPath();
            ctx.moveTo(start.x, start.y);
            ctx.lineTo(end.x, end.y);
            ctx.stroke();
            ctx.setLineDash([]);

            // 如果拖拽距离够远，画方向箭头
            if (dragDist > 8) {
                const angle = Math.atan2(end.y - start.y, end.x - start.x);
                const arrowLen = 16;
                const arrowAngle = 0.6;

                ctx.fillStyle = this.mode === 'set_pose' ? '#6c5ce7' : '#e94560';
                ctx.beginPath();
                ctx.moveTo(end.x, end.y);
                ctx.lineTo(
                    end.x - arrowLen * Math.cos(angle - arrowAngle),
                    end.y - arrowLen * Math.sin(angle - arrowAngle)
                );
                ctx.lineTo(
                    end.x - arrowLen * Math.cos(angle + arrowAngle),
                    end.y - arrowLen * Math.sin(angle + arrowAngle)
                );
                ctx.closePath();
                ctx.fill();
            }

            ctx.restore();
        }

        // --- 模式提示（在画布上） ---
        if (this.mode !== 'view') {
            ctx.save();
            const text = this.mode === 'set_pose' ? '按住拖拽设置初始位姿和方向' : '按住拖拽设置导航目标点和方向';
            ctx.font = 'bold 16px -apple-system, sans-serif';
            ctx.fillStyle = 'rgba(233, 69, 96, 0.9)';
            ctx.textAlign = 'center';
            ctx.fillText(text, w / 2, 30);
            ctx.restore();
        }

        // --- 比例尺 ---
        this.drawScaleBar(ctx);
    },

    // --- 比例尺 ---
    drawScaleBar(ctx) {
        const scaleLen = 1.0;  // 1 米
        const pxLen = scaleLen * this.viewScale;
        if (pxLen < 30 || pxLen > this.width * 0.4) return;

        const x = 12;
        const y = this.height - 24;
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(x, y);
        ctx.lineTo(x + pxLen, y);
        ctx.stroke();

        ctx.beginPath();
        ctx.moveTo(x, y - 6);
        ctx.lineTo(x, y + 6);
        ctx.moveTo(x + pxLen, y - 6);
        ctx.lineTo(x + pxLen, y + 6);
        ctx.stroke();

        ctx.fillStyle = '#ffffff';
        ctx.font = '11px sans-serif';
        ctx.fillText('1 m', x + pxLen / 2 - 12, y - 8);
    },

    // ==================================
    // 交互处理
    // ==================================
    getEventPos(e) {
        const rect = this.canvas.getBoundingClientRect();
        if (e.touches && e.touches.length > 0) {
            return {
                x: e.touches[0].clientX - rect.left,
                y: e.touches[0].clientY - rect.top,
            };
        }
        // changedTouches for touchend
        if (e.changedTouches && e.changedTouches.length > 0) {
            return {
                x: e.changedTouches[0].clientX - rect.left,
                y: e.changedTouches[0].clientY - rect.top,
            };
        }
        return {
            x: e.clientX - rect.left,
            y: e.clientY - rect.top,
        };
    },

    /**
     * 通用的 press 处理：在 set_pose/set_goal 模式下开始放置位姿，
     * 在 view 模式下开始拖拽平移
     */
    _handlePress(pos) {
        if (this.mode === 'set_pose' || this.mode === 'set_goal') {
            // 位姿/目标放置模式：记住起点（地图坐标）
            const mapPos = this.canvasToMap(pos.x, pos.y);
            this.posePlacing = {
                startMapX: mapPos.x,
                startMapY: mapPos.y,
                curMapX: mapPos.x,
                curMapY: mapPos.y,
            };
            this.poseCursorX = pos.x;
            this.poseCursorY = pos.y;
            this.isDragging = false;
        } else {
            // 普通拖拽平移
            this.isDragging = true;
            this.dragStartX = pos.x;
            this.dragStartY = pos.y;
            this.dragStartViewX = this.viewX;
            this.dragStartViewY = this.viewY;
            this._dragMoved = false;
        }
    },

    /**
     * 通用的 move 处理
     */
    _handleMove(pos) {
        if (this.posePlacing) {
            // 正在拖拽放置位姿：更新光标位置
            const mapPos = this.canvasToMap(pos.x, pos.y);
            this.posePlacing.curMapX = mapPos.x;
            this.posePlacing.curMapY = mapPos.y;
            this.poseCursorX = pos.x;
            this.poseCursorY = pos.y;
            this._dragMoved = true;
        } else if (this.isDragging) {
            const dx = pos.x - this.dragStartX;
            const dy = pos.y - this.dragStartY;
            if (Math.abs(dx) > 2 || Math.abs(dy) > 2) {
                this._dragMoved = true;
            }
            this.targetViewX = this.dragStartViewX - dx / this.targetScale;
            this.targetViewY = this.dragStartViewY - dy / this.targetScale;
            this.viewX = this.targetViewX;
            this.viewY = this.targetViewY;
        }
        // 坐标提示回调
        if (this.coordCallback) {
            const mapPos = this.canvasToMap(pos.x, pos.y);
            this.coordCallback(mapPos.x, mapPos.y, pos.x, pos.y);
        }
    },

    /**
     * 通用的 release 处理
     */
    _handleRelease(pos) {
        if (this.posePlacing) {
            // 完成位姿放置
            const startX = this.posePlacing.startMapX;
            const startY = this.posePlacing.startMapY;
            const endX = this.posePlacing.curMapX;
            const endY = this.posePlacing.curMapY;

            // 计算方向角：从起点指向终点
            const dx = endX - startX;
            const dy = endY - startY;
            let yaw = Math.atan2(dy, dx);

            // 如果拖拽距离很短（< 3 像素），视为点击，yaw=0
            const startCanvas = this.mapToCanvas(startX, startY);
            const endCanvas = this.mapToCanvas(endX, endY);
            const dragDist = Math.sqrt(
                (endCanvas.x - startCanvas.x) ** 2 + (endCanvas.y - startCanvas.y) ** 2
            );
            if (dragDist < 3) {
                yaw = 0;
            }

            if (this.mode === 'set_pose' && this.setPoseCallback) {
                this.setPoseCallback(startX, startY, yaw);
            } else if (this.mode === 'set_goal' && this.setGoalCallback) {
                this.setGoalCallback(startX, startY, yaw);
            }

            this.posePlacing = null;
            this._dragMoved = false;
            // 退出放置模式（先保存旧模式再清空）
            const prevMode = this.mode;
            this.mode = 'view';
            this._notifyModeExit(prevMode);
        } else if (this.isDragging && !this._dragMoved) {
            // 短点击（非拖拽）→ 如果是 view 模式什么都不做
        }
        this.isDragging = false;
    },

    /** 通知 UI 退出放置模式 */
    _notifyModeExit(prevMode) {
        if (typeof window !== 'undefined') {
            window.dispatchEvent(new CustomEvent('map_mode_exit', {
                detail: { previousMode: prevMode }
            }));
        }
    },

    onTouchStart(e) {
        e.preventDefault();
        if (e.touches.length === 2) {
            this.isDragging = false;
            this.posePlacing = null;
            const dx = e.touches[0].clientX - e.touches[1].clientX;
            const dy = e.touches[0].clientY - e.touches[1].clientY;
            this.pinchStartDist = Math.sqrt(dx * dx + dy * dy);
            this.pinchStartScale = this.targetScale;
            return;
        }
        if (e.touches.length === 1) {
            const pos = this.getEventPos(e);
            this._handlePress(pos);
        }
    },

    onTouchMove(e) {
        e.preventDefault();
        if (e.touches.length === 2) {
            const dx = e.touches[0].clientX - e.touches[1].clientX;
            const dy = e.touches[0].clientY - e.touches[1].clientY;
            const dist = Math.sqrt(dx * dx + dy * dy);
            if (this.pinchStartDist > 0) {
                const ratio = dist / this.pinchStartDist;
                this.targetScale = Math.max(5, Math.min(200, this.pinchStartScale * ratio));
            }
            return;
        }
        if (e.touches.length === 1) {
            const pos = this.getEventPos(e);
            this._handleMove(pos);
        }
    },

    onTouchEnd(e) {
        if (e.touches.length === 0) {
            const pos = this.getEventPos(e);
            this._handleRelease(pos);
            this.pinchStartDist = 0;
        }
    },

    onMouseDown(e) {
        const pos = this.getEventPos(e);
        this._handlePress(pos);
    },

    onMouseMove(e) {
        const pos = this.getEventPos(e);
        this._handleMove(pos);
    },

    onMouseUp(e) {
        const pos = this.getEventPos(e);
        this._handleRelease(pos);
    },

    onWheel(e) {
        e.preventDefault();

        // 限流：最快每 50ms 处理一次滚轮事件
        const now = Date.now();
        if (now - this.lastWheelTime < 50) return;
        this.lastWheelTime = now;

        // 平滑缩放因子：每次只变 5%
        const factor = e.deltaY < 0 ? 1.05 : 0.95;
        const newScale = Math.max(5, Math.min(200, this.targetScale * factor));

        // 以鼠标/手指位置为中心缩放
        const rect = this.canvas.getBoundingClientRect();
        const mouseX = e.clientX - rect.left;
        const mouseY = e.clientY - rect.top;
        const mapX = (mouseX - this.width / 2) / this.viewScale + this.viewX;
        const mapY = (mouseY - this.height / 2) / this.viewScale + this.viewY;

        this.targetScale = newScale;
        this.targetViewX = mapX - (mouseX - this.width / 2) / newScale;
        this.targetViewY = mapY - (mouseY - this.height / 2) / newScale;
    },
};
