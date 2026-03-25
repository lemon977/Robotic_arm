function(status_json) {
    if (!status_json) return;

    var data;
    try {
        data = JSON.parse(status_json);
    } catch (e) {
        return;
    }

    if (!data || !data.command) return;

    // =====================
    // 初始化全局缓存
    // =====================
    if (!window._lastRosStatus) window._lastRosStatus = {};
    if (!window._lastPyStatus) window._lastPyStatus = {};
    if (!window._videoLoaded) window._videoLoaded = false;
    if (!window._lastArmStatus) {
        window._lastArmStatus = { left: null, right: null };
    }

    // =====================
    // 1️⃣ ROS 脚本指示灯（含左右臂）
    // =====================
    for (var i = 0; i < data.command.length; i++) {
        var item = data.command[i];
        var index = item.index;
        var prev = window._lastRosStatus[index];

        var lamp_dot  = document.getElementById("lamp_dot_" + index);
        var lamp_text = document.getElementById("lamp_text_" + index);
        var lamp_pid  = document.getElementById("lamp_pid_" + index);

        if (!lamp_dot || !lamp_text || !lamp_pid) continue;

        // 申明最初状态
        var color = "gray";
        var statusText = "未启动";

        // ===== 机械臂语义状态 =====
        var armState = null;
        if (i === 4 || i === 5) {
            armState = item.state
        }

        if (item.pid === -1 && !item.alive) {
            color = "gray";
            statusText = "未启动";
        } else if (armState === "zero") {
            color = "yellow";
            statusText = "无动作";
        } else if (armState === "floating") {
            color = "purple";
            statusText = "浮动";
        } else if (item.alive) {
            color = "green";
            statusText = "启动";
            if (item.pid == -1) {
                statusText = "已就绪"
            }
        } else {
            color = "red";
            statusText = "退出";
        }

        lamp_dot.className = "dot " + color;
        lamp_text.innerText = item.name + "-" + statusText;
        lamp_pid.innerText = item.pid === -1 ? "PID: —" : "PID: " + item.pid;

        window._lastRosStatus[index] = {
            alive: item.alive,
            pid: item.pid
        };
    }

    if (data.log){
        log = data.log
        html_log = document.getElementsByClassName('instruct-text')[0].getElementsByTagName("span")[0]
        tag_p = html_log.getElementsByTagName("p")
        
        // console.log(last_p)
        // console.log(log, log[2], html_log)
        if (tag_p.length == 0){
            console.log("tag_p.length == 0")
            new_p = document.createElement("p")
            new_p.innerText = log[log.length-1]
            html_log.appendChild(new_p)
        }else{
            console.log("tag_p.length != 0")
            first_p = tag_p[0]
            last_p = tag_p[tag_p.length-1]
            console.log(first_p.innerText, log[log.length-1], first_p, tag_p)
            // 如果最后一条日志和页面上最后一条日志不一样，就添加新的日志
            if (tag_p.length == 10){
                last_p.remove()
            }
            if (first_p.innerText != log[log.length-1]){
                new_p = document.createElement("p")
                new_p.innerText = log[log.length-1]
                tag_p[0].parentNode.insertBefore(new_p, tag_p[0])
            }
        }
    }


    if(data.test_status){
        test_status = data.test_status
        h5 = document.getElementsByClassName("header")[0].getElementsByTagName("h5")
        h5_1 = h5[0]
        h5_2 = h5[1]

        temp = `当前机器编号:${test_status.machine_name}-${test_status.machine_id} 当前上位机IP:${test_status.model_status.host} 当前上位机端口:${test_status.model_status.port}`
        temp2 = `当前测试模型:${test_status.model_status.model_name} 当前机器状态:${test_status.model_status.run_status}`
        if (h5_1.innerText != temp){
            h5_1.innerText = temp
        }
        if (h5_2.innerText != temp2){
            h5_2.innerText = temp2
        }
    }

    // =====================
    // 3️⃣ 相机视频延迟加载（不变）
    // =====================
    if (!window._videoLoaded && data.command) {
        // console.log("检查 MJPEG 视频流状态...");
        for (var j = 0; j < data.command.length; j++) {
            // console.log("检查 ROS 节点:", data.command[j]);
            if (data.command[j].index === 2 && data.command[j].alive === true) {
                // console.log("加载 MJPEG 视频流...");
                setTimeout(function() {
                    loadMJPEG("cam_l", "http://127.0.0.1:8080/stream?topic=/camera_l/color/image_raw");
                    loadMJPEG("cam_f", "http://127.0.0.1:8080/stream?topic=/camera_f/color/image_raw");
                    loadMJPEG("cam_r", "http://127.0.0.1:8080/stream?topic=/camera_r/color/image_raw");
                }, 1000);
                window._videoLoaded = true;
            }
        }
    }

    function loadMJPEG(imgId, url) {
        var img = document.getElementById(imgId);
        if (!img) {
            console.warn(`找不到 ID 为 ${imgId} 的 img 元素，终止加载`);
            return;
        }

        // 初始化 img 样式（隐藏、清空原有 src）
        img.style.opacity = "0";
        img.src = "";

        // 定义重试定时器（用于后续清除，避免内存泄漏）
        var retryTimer = null;
        // 重试间隔（1 秒）
        var retryInterval = 1000;

        // 封装 img 重试加载逻辑（方便定时器重复调用）
        function retryLoadImg() {
            // 重新设置 img.src 发起加载（MJPEG 流直接赋值即可重试）
            img.src = url;
        }

        // 1. 给页面实际 img 绑定加载成功事件（加载好就停止重试，保持当前状态）
        img.onload = function() {
            console.log(`[${imgId}] MJPEG 流加载成功，无需再重试`);
            // 清除重试定时器，终止后续所有重试
            if (retryTimer) {
                clearInterval(retryTimer);
                retryTimer = null;
            }
            // 确保样式正常（和原有逻辑保持一致）
            img.style.width = "100%";
            img.style.height = "100%";
            img.style.opacity = "1";
        };

        // 2. 给页面实际 img 绑定加载失败事件（加载失败触发定时重试）
        img.onerror = function() {
            console.warn(`[${imgId}] MJPEG 流加载失败，${retryInterval/1000} 秒后重试...`);
            // 关键：确保定时器只创建一次，避免多个定时器导致重试频率异常
            if (!retryTimer) {
                retryTimer = setInterval(retryLoadImg, retryInterval);
            }
        };

        // 3. 原有 probe 预加载逻辑（验证 URL 可达性，避免无效重试）
        var probe = new Image();
        probe.onload = function() {
            console.log(`[${imgId}] MJPEG 流 URL 可达，开始初始化 img 加载`);
            // probe 验证通过后，首次给 img 赋值 src，启动后续加载/重试流程
            retryLoadImg();
        };

        // probe 预加载失败（URL 不可达，直接终止，无需重试）
        probe.onerror = function() {
            console.error(`[${imgId}] MJPEG 流 URL 不可达，无法进行后续加载和重试`);
        };

        // 发起 probe 预验证
        probe.src = url;
    }



    // =====================
    // ⌨️ 键盘监听（只注册一次）
    // =====================
    // if (!window._keyListenerRegistered) {
    //     window._keyListenerRegistered = true;

    //     document.addEventListener("keydown", function(e) {
    //         console.log("按键事件:", e.key, e.code);

    //         if (e.code === "Enter") {
    //             e.preventDefault();
    //             var isHidden = document.getElementById("replay_popup").classList.contains("hide")
    //             if (isHidden){
    //                 document.getElementById("start_collect_btn")?.click();
    //                 console.log("触发开始采集按钮点击");
    //             }else{
    //                 op_tips = document.getElementById("op_tips")
    //                 text = op_tips.querySelector("textarea")
    //                 // console.log("当前提示文本:", text.value)
    //                 text.value = "当前已打开重播面板，请关闭后再进行数据采集！！！"
    //             }
    //         }

    //         if (e.code === "Space") {
    //             e.preventDefault();
    //             document.getElementById("end_collect_btn")?.click();
    //             console.log("触发结束采集按钮点击");
    //         }

    //         if (e.code === "Backspace") {
    //             e.preventDefault();
    //             document.getElementById("stop_collect_btn")?.click();
    //             console.log("触发重播按钮点击");
    //         }
    //     });
    // }


    // if (!window._configPanelAutoClicked) {
    //     window._configPanelAutoClicked = true;
    //     var panel = document.getElementById("config_btn");
    //     if (panel) {
    //         panel.click();
    //     }
    // }


}