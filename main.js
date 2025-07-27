document.addEventListener('DOMContentLoaded', () => {
    // --- 初期設定 ---
    const canvas = document.getElementById('simulationCanvas');
    const ctx = canvas.getContext('2d');
    const startButton = document.getElementById('startButton');
    const resetButton = document.getElementById('resetButton');
    let path = [];
    let robot = null;
    let simulationRunning = false;
    let animationFrameId = null;
    let lastLookedSegmentIndex = 0;

    // --- ロボットクラスの定義 (X字オムニホイール) ---
    class OmniRobot {
        constructor(x, y, angle = 0) {
            this.x = x; this.y = y; this.angle = angle;
            this.radius = 25; // ロボット全体の半径として使う
            this.maxSpeed = 2.0; this.turnSpeed = 0.05;
            this.current_vx = 0; this.current_vy = 0;

            // オムニホイール関連の描画プロパティ
            this.bodyRadius = 17;
            this.armLength = this.radius;
            this.wheelRadius = 8;
            this.wheelPositions = [];
            this.wheelVelocities = []; // 各ホイールの速度ベクトルを格納
            for(let i = 0; i < 4; i++) {
                const angle = (Math.PI / 4) + (i * Math.PI / 2); // 45度ずつ回転
                this.wheelPositions.push({
                    x: this.armLength * Math.cos(angle),
                    y: this.armLength * Math.sin(angle)
                });
                this.wheelVelocities.push({vx: 0, vy: 0});
            }
        }

        update(vx, vy, omega) {
            this.current_vx = vx; this.current_vy = vy;

            // ワールド座標系での移動量を計算
            const world_vx = vx * Math.cos(this.angle) - vy * Math.sin(this.angle);
            const world_vy = vx * Math.sin(this.angle) + vy * Math.cos(this.angle);
            
            // 位置と角度を更新
            this.x += world_vx;
            this.y += world_vy;
            this.angle += omega;
            this.angle = (this.angle + Math.PI) % (2 * Math.PI) - Math.PI; // -πから+πの範囲に正規化

            // 各ホイールの速度ベクトルを計算（シミュレーション用）
            // これは運動学の逆解き（インバースキネマティクス）の簡易版
            const L = this.armLength; // 中心からホイールまでの距離
            const r = this.wheelRadius; // ホイール半径（計算には使わないが概念として）
            
            // ロボット座標系での目標速度
            const robot_vx = vx;
            const robot_vy = vy;
            const robot_omega = omega;

            // 各ホイールが出すべき接線速度を計算
            // v_wheel = [-sin(theta) cos(theta)] * [vx; vy] + L * omega
            this.wheelPositions.forEach((pos, i) => {
                const theta = Math.atan2(pos.y, pos.x); // 各ホイールの取り付け角度
                const v_tangential = -Math.sin(theta) * robot_vx + Math.cos(theta) * robot_vy + L * robot_omega * 3; // 回転を強調
                
                // 接線速度をベクトルに変換（ローラーの方向）
                const roller_angle = theta + Math.PI / 2; // 接線方向
                this.wheelVelocities[i] = {
                    vx: v_tangential * Math.cos(roller_angle),
                    vy: v_tangential * Math.sin(roller_angle)
                };
            });
        }

        draw() {
            ctx.save();
            ctx.translate(this.x, this.y);
            ctx.rotate(this.angle);

            // アームの描画
            ctx.strokeStyle = '#6c757d';
            ctx.lineWidth = 5;
            this.wheelPositions.forEach(pos => {
                ctx.beginPath();
                ctx.moveTo(0, 0);
                ctx.lineTo(pos.x, pos.y);
                ctx.stroke();
            });
            
            // ホイールの描画と速度矢印の描画
            this.wheelPositions.forEach((pos, i) => {
                // ホイール本体
                ctx.fillStyle = '#343a40';
                ctx.beginPath();
                ctx.arc(pos.x, pos.y, this.wheelRadius, 0, 2 * Math.PI);
                ctx.fill();

                // ホイールのローラー（簡易表現）
                ctx.strokeStyle = '#adb5bd';
                ctx.lineWidth = 1.5;
                ctx.beginPath();
                ctx.moveTo(pos.x - this.wheelRadius, pos.y);
                ctx.lineTo(pos.x + this.wheelRadius, pos.y);
                ctx.moveTo(pos.x, pos.y - this.wheelRadius);
                ctx.lineTo(pos.x, pos.y + this.wheelRadius);
                ctx.stroke();
                
                // 速度ベクトルの矢印を描画
                const vel = this.wheelVelocities[i];
                const scale = 15; // 矢印の長さのスケール
                const arrowStartX = pos.x;
                const arrowStartY = pos.y;
                const arrowEndX = pos.x + vel.vx * scale;
                const arrowEndY = pos.y + vel.vy * scale;

                ctx.strokeStyle = '#007bff';
                ctx.lineWidth = 2;
                ctx.beginPath();
                ctx.moveTo(arrowStartX, arrowStartY);
                ctx.lineTo(arrowEndX, arrowEndY);
                ctx.stroke();
                
                // 矢印の先端
                const headlen = 5;
                const angle = Math.atan2(arrowEndY - arrowStartY, arrowEndX - arrowStartX);
                ctx.fillStyle = '#007bff';
                ctx.beginPath();
                ctx.moveTo(arrowEndX, arrowEndY);
                ctx.lineTo(arrowEndX - headlen * Math.cos(angle - Math.PI / 6), arrowEndY - headlen * Math.sin(angle - Math.PI / 6));
                ctx.lineTo(arrowEndX - headlen * Math.cos(angle + Math.PI / 6), arrowEndY - headlen * Math.sin(angle + Math.PI / 6));
                ctx.closePath();
                ctx.fill();
            });

            // 中央のボディ
            ctx.fillStyle = '#007bff';
            ctx.beginPath();
            ctx.arc(0, 0, this.bodyRadius, 0, 2 * Math.PI);
            ctx.fill();

            ctx.restore();
        }
    }

    // --- Pure Pursuit アルゴリズム ---
    function purePursuit() {
        if (!robot || path.length < 2) {
            return { control: { vx: 0, vy: 0, omega: 0 }, debug: {} };
        }

        const k = 10.0; const minLookahead = 15.0; const maxLookahead = 50.0;
        const currentSpeed = Math.hypot(robot.current_vx, robot.current_vy);
        let lookaheadDistance = currentSpeed * k + minLookahead;
        lookaheadDistance = Math.max(minLookahead, Math.min(lookaheadDistance, maxLookahead));

        let targetPoint = null;
        let targetSegmentIndex = lastLookedSegmentIndex;

        for (let i = lastLookedSegmentIndex; i < path.length - 1; i++) {
            const p1 = path[i];
            const p2 = path[i + 1];
            const intersections = findCircleLineIntersections(robot, lookaheadDistance, p1, p2);

            if (intersections.length > 0) {
                let forwardIntersections = [];
                const robotHeadingVec = { x: Math.cos(robot.angle), y: Math.sin(robot.angle) };

                for (const p of intersections) {
                    const vecToPoint = { x: p.x - robot.x, y: p.y - robot.y };
                    const dotProduct = robotHeadingVec.x * vecToPoint.x + robotHeadingVec.y * vecToPoint.y;
                    if (dotProduct > 0) {
                        forwardIntersections.push(p);
                    }
                }

                if (forwardIntersections.length > 0) {
                    let maxDist = -1;
                    for (const p of forwardIntersections) {
                        const d = Math.hypot(p.x - robot.x, p.y - robot.y);
                        if (d > maxDist) {
                            maxDist = d;
                            targetPoint = p;
                        }
                    }
                    targetSegmentIndex = i;
                    lastLookedSegmentIndex = i;
                    break;
                }
            }
        }

        let final_local_vx = 0, final_local_vy = 0, final_omega = 0;

        if (targetPoint) {
            // [通常時] 目標点に向かう制御
            const world_dx = targetPoint.x - robot.x;
            const world_dy = targetPoint.y - robot.y;
            const world_target_angle = Math.atan2(world_dy, world_dx);
            const desired_world_vx = robot.maxSpeed * Math.cos(world_target_angle);
            const desired_world_vy = robot.maxSpeed * Math.sin(world_target_angle);
            final_local_vx = desired_world_vx * Math.cos(-robot.angle) - desired_world_vy * Math.sin(-robot.angle);
            final_local_vy = desired_world_vx * Math.sin(-robot.angle) + desired_world_vy * Math.cos(-robot.angle);

            // P制御による角度制御
            const prevPathSegment = path[targetSegmentIndex];
            const nextPathSegment = path[targetSegmentIndex + 1] ? path[targetSegmentIndex + 1] : prevPathSegment;
            const pathAngle = Math.atan2(nextPathSegment.y - prevPathSegment.y, nextPathSegment.x - prevPathSegment.x);
            
            let angle_error = pathAngle - robot.angle;
            angle_error = (angle_error + Math.PI) % (2 * Math.PI) - Math.PI;
            
            const p_gain = 0.5;
            let omega_command = angle_error * p_gain;

            const angleTolerance = 0.035;
            if (Math.abs(angle_error) < angleTolerance) omega_command = 0;
            
            final_omega = Math.max(-robot.turnSpeed, Math.min(robot.turnSpeed, omega_command));

        } else {
            // [最終地点接近時] 目標点が見つからない場合、終点に向かう
            targetPoint = path[path.length - 1];
            const dist = Math.hypot(robot.x - targetPoint.x, robot.y - targetPoint.y);
            const arrivalThreshold = robot.radius * 0.25;
            if (dist < arrivalThreshold) {
                return { control: { vx: 0, vy: 0, omega: 0 }, debug: {} };
            }

            const arrivalRadius = 150.0;
            let desiredSpeed = robot.maxSpeed;
            if (dist < arrivalRadius) {
                desiredSpeed = robot.maxSpeed * (dist / arrivalRadius);
            }
            desiredSpeed = Math.max(desiredSpeed, 0.05);

            const world_dx = targetPoint.x - robot.x;
            const world_dy = targetPoint.y - robot.y;
            const world_target_angle = Math.atan2(world_dy, world_dx);
            const desired_world_vx = desiredSpeed * Math.cos(world_target_angle);
            const desired_world_vy = desiredSpeed * Math.sin(world_target_angle);
            final_local_vx = desired_world_vx * Math.cos(-robot.angle) - desired_world_vy * Math.sin(-robot.angle);
            final_local_vy = desired_world_vx * Math.sin(-robot.angle) + desired_world_vy * Math.cos(-robot.angle);
            final_omega = 0;
        }

        return {
            control: { vx: final_local_vx, vy: final_local_vy, omega: final_omega },
            debug: { lookaheadDistance, targetPoint }
        };
    }

    // --- ヘルパー関数: 円と線分の交点計算 ---
    function findCircleLineIntersections(center, radius, p1, p2) {
        let dx = p2.x - p1.x; let dy = p2.y - p1.y;
        let fx = p1.x - center.x; let fy = p1.y - center.y;
        let a = dx * dx + dy * dy;
        let b = 2 * (fx * dx + fy * dy);
        let c = (fx * fx + fy * fy) - radius * radius;
        let discriminant = b * b - 4 * a * c;
        let intersections = [];
        if (discriminant >= 0) {
            discriminant = Math.sqrt(discriminant);
            let t1 = (-b - discriminant) / (2 * a);
            let t2 = (-b + discriminant) / (2 * a);
            if (t1 >= 0 && t1 <= 1) {
                intersections.push({ x: p1.x + t1 * dx, y: p1.y + t1 * dy });
            }
            if (t2 >= 0 && t2 <= 1) {
                intersections.push({ x: p1.x + t2 * dx, y: p1.y + t2 * dy });
            }
        }
        return intersections;
    }

    // --- 描画とシミュレーションループ ---
    function draw(debugInfo = {}) {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        // パスを描画
        if (path.length > 1) {
            ctx.strokeStyle = '#6c757d';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(path[0].x, path[0].y);
            for (let i = 1; i < path.length; i++) {
                ctx.lineTo(path[i].x, path[i].y);
            }
            ctx.stroke();
        }
        // ウェイポイントを描画
        ctx.fillStyle = '#17a2b8';
        path.forEach(p => {
            ctx.beginPath();
            ctx.arc(p.x, p.y, 5, 0, 2 * Math.PI);
            ctx.fill();
        });
        // ロボットとデバッグ情報を描画
        if (robot) {
            if (simulationRunning && debugInfo.lookaheadDistance) {
                // 先行探索円を描画
                ctx.strokeStyle = 'rgba(255, 0, 0, 0.4)';
                ctx.lineWidth = 1;
                ctx.beginPath();
                ctx.arc(robot.x, robot.y, debugInfo.lookaheadDistance, 0, 2 * Math.PI);
                ctx.stroke();
                // 目標点を描画
                if (debugInfo.targetPoint) {
                    ctx.fillStyle = 'rgba(255, 0, 0, 0.8)';
                    ctx.beginPath();
                    ctx.arc(debugInfo.targetPoint.x, debugInfo.targetPoint.y, 8, 0, 2 * Math.PI);
                    ctx.fill();
                }
            }
            robot.draw();
        }
    }

    function animate() {
        let debugData = {};
        if (simulationRunning && robot) {
            const result = purePursuit();
            robot.update(result.control.vx, result.control.vy, result.control.omega);
            debugData = result.debug;
        }
        draw(debugData);
        animationFrameId = requestAnimationFrame(animate);
    }

    // --- イベントリスナー ---
    canvas.addEventListener('click', (e) => {
        if (simulationRunning) return;
        const rect = canvas.getBoundingClientRect();
        const scaleX = canvas.width / rect.width;
        const scaleY = canvas.height / rect.height;
        const x = (e.clientX - rect.left) * scaleX;
        const y = (e.clientY - rect.top) * scaleY;
        path.push({ x, y });
    });

    startButton.addEventListener('click', () => {
        if (path.length > 0 && robot) {
            simulationRunning = !simulationRunning;
            startButton.textContent = simulationRunning ? 'Stop' : 'Start';
        }
    });

    resetButton.addEventListener('click', () => {
        simulationRunning = false;
        startButton.textContent = 'Start';
        path = [];
        robot = new OmniRobot(canvas.width / 2, canvas.height - 50, -Math.PI / 2);
        lastLookedSegmentIndex = 0;
    });

    // --- 初期化処理 ---
    function init() {
        robot = new OmniRobot(canvas.width / 2, canvas.height - 50, -Math.PI / 2);
        animate();
    }

    init();
});