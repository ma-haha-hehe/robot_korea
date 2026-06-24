import pyrealsense2 as rs
try:
    ctx = rs.context()
    devices = ctx.query_devices()
    print(f"找到设备数量: {len(devices)}")
    if len(devices) > 0:
        pipeline = rs.pipeline()
        pipeline.start()
        print("Pipeline 启动成功！硬件连接正常。")
        pipeline.stop()
    else:
        print("SDK 仍然看不见任何设备。")
except Exception as e:
    print(f"错误: {e}")