from flask import Flask, jsonify, request
import json
import datetime
import os, re
from flask_cors import CORS

# 初始化 Flask 应用
app = Flask(__name__)
CORS(app)  # 启用 CORS
log_url_replay = '/home/jinglin/json_server/log/replay'
if not os.path.exists(log_url_replay):
    os.mkdir(log_url_replay)
log_url_collect = '/home/jinglin/json_server/log/collect'
if not os.path.exists(log_url_collect):
    os.mkdir(log_url_collect)
log_url_delete = '/home/jinglin/json_server/log/delete'
if not os.path.exists(log_url_delete):
    os.mkdir(log_url_delete)


# 总体读取json函数
def read_json(value, file_name, before_key, after_key):
    """
        value: 传入的值,
        file_name: 需要读取的json文件名
        before_key: 传入的值的称呼
        after_key: 传出的值的称呼
        例如 before_key -> ip  after_key -> id 为 通过ip获取id
    """
    if value == -1:
        return ({
            'code': 201,
            'message': f'{before_key} is error',
            'data': None
        })

    with open(f'/home/jinglin/json_server/{file_name}.json', 'r') as f:
        text = json.load(f)
        try:
            data = text
            if value:
                data = text[value]
            return ({
                'code': 200,
                'message': f'get {after_key} success',
                'data': data
            })
        except Exception as e:
            return ({
                'code': 202,
                'message': f'{before_key} is not register',
                'data': None
            })


# 通过ip获取机器编号
@app.route('/api/get_id', methods=['GET'])
def get_id():
    ip = request.args.get('ip', -1)
    return read_json(ip, 'get_id', 'ip', 'id')


# 通过机器编号获取具体任务
@app.route('/api/get_tasks', methods=['GET'])
def get_tasks():
    machine_id = request.args.get('id', -1)
    return read_json(machine_id, 'machine_info', 'id', 'tasks')


# 获取所有用户的任务（管理端/管理人员局域网访问json文件使用）
@app.route('/api/get_all_tasks', methods=['GET'])
def get_all_tasks():
    return read_json(None, 'machine_info', 'all', 'tasks')


# 重播结束后上传当前重播的数据，如果数据不重复，那么进行重播数量的更新
@app.route('/api/add_replay_data', methods=['POST'])
def receive_data():
    try:
        # 获取 POST 请求的 JSON 数据
        data = request.get_json()
        if not data:
            return jsonify({'code': 201, 'message': 'No JSON data provided'})
        
        dataset_dir = data['dataset_dir']
        replay_idx = data['replay_idx'] 
        log_name = dataset_dir.split('/')[-1]
        log_dir = f'{log_url_replay}/{log_name}.log'
                          
        if os.path.exists(log_dir):
            with open(log_dir, 'r') as f:
                existing_replay_idxs = set(line.strip().split(': ')[-1] for line in f)
            if replay_idx in existing_replay_idxs:
                return jsonify({'code': 202, 'message': 'Replay data already exists'})
            else:
                with open(log_dir, 'a', encoding='utf-8') as f:
                    f.write(f'{datetime.datetime.now()}: {dataset_dir}/{replay_idx}\n')
                return jsonify({'code': 200, 'message': 'add_replay_data success'})
        else:
            with open(log_dir, 'w', encoding='utf-8') as f:
                f.write(f'{datetime.datetime.now()}: {dataset_dir}/{replay_idx}\n')
            return jsonify({'code': 200, 'message': 'add_replay_data success'})
    except Exception as e:
        return jsonify({'code': 202, 'message': f'Error: {str(e)}'})
    

# 查询重播数据，返回重播数据列表
@app.route('/api/get_replay_data', methods=['POST'])
def get_replay_data():
    try:
        data = request.get_json()
        if not data:
            return jsonify({'code': 201, 'message': 'No JSON data provided'})
        
        dataset_dir = data['dataset_dir']
        log_name = dataset_dir.split('/')[-1]
        log_dir = f'{log_url_replay}/{log_name}.log'
        if os.path.exists(log_dir):
            with open(log_dir, 'r') as f:
                replay_data = [re.compile(r'.*episode_(\d+)\.hdf5').match(line.strip()).group(1) for line in f if re.compile(r'.*episode_(\d+)\.hdf5').match(line.strip())]
            return jsonify({'code': 200, 'message': 'get_replay_data success', 'data': replay_data})
        else:
            return jsonify({'code': 201, 'message': 'No replay data found', 'data': []})
    
    except Exception as e:
        return jsonify({'code': 202, 'message': f'Error: {str(e)}'})


# 新增采集记录
@app.route('/api/add_collect_data', methods=['POST'])
def add_collect_data():
    try:
        data = request.get_json()
        if not data:
            return jsonify({'code': 201, 'message': 'No JSON data provided'})
        
        dataset_dir = data['dataset_dir']
        collect_idx = data['ep_idx']
        log_name = dataset_dir.split('/')[-1]
        log_dir = f'{log_url_collect}/{log_name}.log'

        if os.path.exists(log_dir):
            with open(log_dir, 'r') as f:
                existing_collect_idxs = set(line.strip().split(': ')[-1] for line in f)
            if collect_idx in existing_collect_idxs:
                return jsonify({'code': 202, 'message': 'Collect data already exists'})
            else:
                with open(log_dir, 'a', encoding='utf-8') as f:
                    f.write(f'{datetime.datetime.now()}: {dataset_dir}/episode_{collect_idx}.hdf5\n')
                return jsonify({'code': 200, 'message': 'add_collect_data success'})
        else:
            with open(log_dir, 'w', encoding='utf-8') as f:
                f.write(f'{datetime.datetime.now()}: {dataset_dir}/episode_{collect_idx}.hdf5\n')
            return jsonify({'code': 200, 'message': 'add_collect_data success'})
    except Exception as e:
        return jsonify({'code': 202, 'message': f'Error: {str(e)}'})


# 查询采集记录
@app.route('/api/get_collect_data', methods=['POST'])
def get_collect_data():
    try:
        data = request.get_json()
        if not data:
            return jsonify({'code': 201, 'message': 'No JSON data provided'})
        
        dataset_dir = data['dataset_dir']
        log_name = dataset_dir.split('/')[-1]
        log_dir = f'{log_url_collect}/{log_name}.log'
        if os.path.exists(log_dir):
            with open(log_dir, 'r') as f:
                collect_data = [re.compile(r'.*episode_(\d+)\.hdf5').match(line.strip()).group(1) for line in f if re.compile(r'.*episode_(\d+)\.hdf5').match(line.strip())]
            return jsonify({'code': 200, 'message': 'get_collect_data success', 'data': collect_data})
        else:
            return jsonify({'code': 201, 'message': 'No collect data found', 'data': []})
    
    except Exception as e:
        return jsonify({'code': 202, 'message': f'Error: {str(e)}'})
    

# 删除采集记录
@app.route('/api/delete_host_data', methods=['POST'])
def delete_host_data():
    try:
        data = request.get_json()
        if not data:
            return jsonify({'code': 201, 'message': 'No JSON data provided'})
        
        dataset_dir = data['dataset_dir']
        idx = data['idx']
        log_name = dataset_dir.split('/')[-1]
        log_collect_dir = f'{log_url_collect}/{log_name}.log'
        log_replay_dir = f'{log_url_replay}/{log_name}.log'
        log_delete_dir = f'{log_url_delete}/{log_name}.log'

        collect_to_keep = []
        replay_to_keep = []
        if os.path.exists(log_collect_dir):
            with open(log_collect_dir, 'r') as f:
                for line in f:
                    if not (f'episode_{idx}.hdf5' in line.strip().split(': ')[-1]):
                        collect_to_keep.append(line.strip())
            with open(log_collect_dir, 'w', encoding='utf-8') as f:
                for line in collect_to_keep:
                    f.write(f'{line}\n')

        if os.path.exists(log_replay_dir):
            with open(log_replay_dir, 'r') as f:
                for line in f:
                    if not (f'episode_{idx}.hdf5' in line.strip().split(': ')[-1]):
                        replay_to_keep.append(line.strip())
            with open(log_replay_dir, 'w', encoding='utf-8') as f:
                for line in replay_to_keep:
                    f.write(f'{line}\n')
        with open(log_delete_dir, 'a', encoding='utf-8') as f:
            f.write(f'{datetime.datetime.now()}: {dataset_dir}/episode_{idx}.hdf5\n')
            
        return jsonify({'code': 200, 'message': 'delete_host_data success'})
    except Exception as e:
        return jsonify({'code': 202, 'message': f'Error: {str(e)}'})


if __name__ == '__main__':
    # 关键配置：
    # host='0.0.0.0' 表示监听所有网络接口（局域网可访问）
    # port 自定义端口（如 5000，可修改）
    # debug=True 开发模式（生产环境请改为 False）
    app.run(host='0.0.0.0', port=5000, debug=False)
