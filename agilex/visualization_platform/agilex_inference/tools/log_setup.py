import logging
import logging.handlers
import os
from datetime import datetime

def setup_logger(
    name: str = "my_app",
    log_dir: str = "./logs",
    level: int = logging.INFO,
    max_bytes: int = 10 * 1024 * 1024,  # 单个日志文件最大10MB
    backup_count: int = 5,  # 保留5个备份文件
    log_format: str = "%(asctime)s - %(name)s - %(levelname)s - %(filename)s:%(lineno)d - %(message)s"
) -> logging.Logger:
    """
    配置并返回一个日志器
    
    Args:
        name: 日志器名称
        log_dir: 日志文件存储目录
        level: 日志级别
        max_bytes: 单个日志文件最大字节数
        backup_count: 备份文件数量
        log_format: 日志格式
    
    Returns:
        配置好的Logger对象
    """
    # 1. 创建日志器，避免重复添加处理器
    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.handlers.clear()  # 清除已存在的处理器，防止重复输出
    
    # 2. 创建日志目录
    os.makedirs(log_dir, exist_ok=True)
    
    # 3. 定义日志格式
    formatter = logging.Formatter(log_format, datefmt="%Y-%m-%d %H:%M:%S")
    
    # 4. 控制台处理器（输出到屏幕）
    console_handler = logging.StreamHandler()
    console_handler.setLevel(level)
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)
    
    # 5. 文件处理器（按大小分割）
    # log_filename = os.path.join(log_dir, f"{name}_{datetime.now().strftime('%Y%m%d')}.log")
    log_filename = os.path.join(log_dir, f"{name}.log")
    file_handler = logging.handlers.RotatingFileHandler(
        filename=log_filename,
        maxBytes=max_bytes,
        backupCount=backup_count,
        encoding="utf-8"  # 确保中文正常显示
    )
    file_handler.setLevel(level)
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)
    
    return logger