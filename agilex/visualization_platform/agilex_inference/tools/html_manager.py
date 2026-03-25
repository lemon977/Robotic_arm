import json

# ========= 读取HTML文件函数 ==========
def read_html_file(file_path):
    """
    该函数读取指定路径的HTML文件，并返回文件内容的字符串。可以根据需要对文件内容进行修改或处理。
        - file_path: HTML文件的路径，包含需要读取的HTML内容。
    返回值:
        - 包含HTML文件内容的字符串，以便在前端更新UI显示。
    """
    txt = ""
    with open(file_path, 'r') as f:
        txt = f.read()
    return txt


# ========= 更新UI数据 ==========
def update_ui(COMMANDS_STATUS, THIS_LOG=None, TEST_STATUS=None, MODEL_STATUS=None):
    """
    该函数将COMMANDS_STATUS、TEST_STATUS、MODEL_STATUS和THIS_LOG等数据进行处理，并返回一个包含这些数据的JSON字符串，以便在前端更新UI显示。可以根据需要对数据进行修改或处理。
        - COMMANDS_STATUS: 包含命令状态的列表，每个元素是一个字典，包含命令的相关信息。
        - THIS_LOG: 包含日志信息的列表，每个元素是一个字符串，表示一条日志记录。
        - TEST_STATUS: 包含测试状态的字典，包含测试的相关信息。
        - MODEL_STATUS: 包含模型状态的字典，包含模型的相关信息。
    返回值:
        - 包含COMMANDS_STATUS、TEST_STATUS、MODEL_STATUS和THIS_LOG等数据的JSON字符串，以便在前端更新UI显示。
    """
    # 将Porocess对象移除，避免序列化失败
    command_results = [
        { k: v for k, v in item.items() if k != "process" and k != "ssh_client" and k != "channel" }
        for item in COMMANDS_STATUS
    ]

    model_status = { k: v for k, v in MODEL_STATUS.items() if k != "ssh_client" and k != "channel" } if MODEL_STATUS else {}

    test_status = { k: v for k, v in TEST_STATUS.items() if k != "process" } if TEST_STATUS else {}

    test_status.update({
        "model_status": model_status if model_status else {}
    })

    # print("ui数据更新：", test_status)
    # print("更新UI数据：", command_results, setup_data, THIS_LOG)
    data = {
        "command": command_results,
        "test_status": test_status if test_status else {},
        "log": THIS_LOG if THIS_LOG is not None else []
    }

    return [json.dumps(data), json.dumps(model_status) if model_status else json.dumps({})]


# ========= 更新指示器JS ==========
def update_indicator_js(html_js_path):
    """
    该函数读取HTML文件中的JS代码，并返回JS代码字符串，以便在前端更新指示器显示。可以根据需要对JS代码进行修改或处理。
        - html_js_path: HTML文件的路径，包含需要读取的JS代码。
    返回值:
        - 包含JS代码的字符串，以便在前端更新指示器显示。
    """
    print(html_js_path)
    js = read_html_file(html_js_path)
    return js 