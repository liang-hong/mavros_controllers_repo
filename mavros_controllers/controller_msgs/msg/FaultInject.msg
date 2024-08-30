# 自定义消息类型 FaultInject

# 指示是否发生故障, 同时也是故障类型序号, 0为无故障
uint8 fault_occur

# 四个浮点数值，每个值的范围从 0 到 2, 1表示无故障
float32[4] fault_values  # 每个元素的值域 [0, 2]
