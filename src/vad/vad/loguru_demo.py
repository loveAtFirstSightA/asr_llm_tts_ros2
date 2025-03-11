from loguru import logger
import sys

# 初始化 loguru
# 删除默认的 logger 配置
logger.remove()

# 添加自定义配置：输出到标准输出，日志级别为 DEBUG，并指定日志格式
logger.add(sys.stdout, level="DEBUG", format="{time:YYYY-MM-DD HH:mm:ss} [ {level} ]{message}")

def main():
    # 打印字符串
    logger.info("Hello, world!")
    
    # 打印整数和浮点数
    logger.info("Integer: {}", 42)
    logger.info("Float: {}", 3.14159)
    
    # 打印列表
    my_list = [1, 2, 3, "a", "b", "c"]
    logger.info("List: {}", my_list)
    
    # 打印字典
    my_dict = {"name": "Alice", "age": 30}
    logger.info("Dictionary: {}", my_dict)
    
    # 打印元组
    my_tuple = (10, 20, 30)
    logger.info("Tuple: {}", my_tuple)
    
    # 打印集合（无序且自动去重）
    my_set = {1, 2, 3, 3, 2, 1}
    logger.info("Set: {}", my_set)
    
    # 打印嵌套数据结构
    nested = {"list": [1, {"key": "value"}, (2, 3)], "number": 100}
    logger.info("Nested structure: {}", nested)
    
    # 捕获并打印异常信息
    try:
        result = 1 / 0
    except Exception as e:
        logger.exception("An exception occurred: {}", e)

if __name__ == "__main__":
    main()
