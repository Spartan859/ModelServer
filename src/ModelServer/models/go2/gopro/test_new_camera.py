#!/usr/bin/env python3
"""
测试新的多进程相机接口
"""

import time
import cv2
import logging
from gopro10 import USBCamera, detect_cameras

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def test_camera_performance():
    """测试相机性能"""
    logger.info("开始测试新的多进程相机接口...")
    
    # 检测相机
    cameras = detect_cameras()
    if not cameras:
        logger.error("未检测到任何相机设备")
        return
    
    camera_id = cameras[0]
    logger.info(f"使用相机设备: {camera_id}")
    
    # 创建相机实例
    camera = USBCamera(device_id=camera_id, width=640, height=480, fps=30)
    
    try:
        # 打开相机
        if not camera.open():
            logger.error("无法打开相机")
            return
        
        # 显示相机信息
        info = camera.get_camera_info()
        logger.info(f"相机信息: {info}")
        
        # 等待子进程稳定
        time.sleep(1.0)
        
        # 性能测试
        frame_count = 0
        start_time = time.time()
        test_duration = 10.0  # 测试10秒
        
        logger.info(f"开始 {test_duration} 秒性能测试...")
        
        while time.time() - start_time < test_duration:
            frame = camera.read_frame()
            
            if frame is not None:
                frame_count += 1
                
                # 每秒显示一次统计
                elapsed = time.time() - start_time
                if frame_count % 30 == 0:  # 约每秒显示一次
                    fps = frame_count / elapsed
                    queue_info = camera.get_camera_info()
                    logger.info(f"已读取 {frame_count} 帧, FPS: {fps:.1f}, "
                              f"进程状态: {queue_info.get('process_alive', False)}, "
                              f"队列大小: {queue_info.get('queue_size', 0)} (最大1帧)")
            else:
                logger.warning("读取帧失败")
                time.sleep(0.1)  # 失败时稍作等待
        
        # 最终统计
        total_time = time.time() - start_time
        average_fps = frame_count / total_time
        
        logger.info(f"测试完成!")
        logger.info(f"总时间: {total_time:.2f}s")
        logger.info(f"总帧数: {frame_count}")
        logger.info(f"平均FPS: {average_fps:.2f}")
        logger.info(f"理论最大FPS: 30")
        
        # 测试延迟
        logger.info("测试读取延迟...")
        latency_tests = []
        for i in range(10):
            start = time.time()
            frame = camera.read_frame()
            end = time.time()
            if frame is not None:
                latency_tests.append((end - start) * 1000)  # 转换为毫秒
        
        if latency_tests:
            avg_latency = sum(latency_tests) / len(latency_tests)
            min_latency = min(latency_tests)
            max_latency = max(latency_tests)
            logger.info(f"读取延迟 - 平均: {avg_latency:.2f}ms, "
                       f"最小: {min_latency:.2f}ms, 最大: {max_latency:.2f}ms")
    
    except KeyboardInterrupt:
        logger.info("用户中断测试")
    
    except Exception as e:
        logger.error(f"测试过程中发生错误: {e}")
    
    finally:
        # 关闭相机
        camera.close()
        logger.info("测试结束")

def test_visual_display():
    """可视化测试"""
    logger.info("开始可视化测试...")
    
    cameras = detect_cameras()
    if not cameras:
        logger.error("未检测到任何相机设备")
        return
    
    with USBCamera(device_id=cameras[0], width=640, height=480, fps=30) as camera:
        if not camera.is_opened:
            logger.error("无法打开相机")
            return
        
        logger.info("按 'q' 键退出, 按 's' 键保存图片")
        
        frame_count = 0
        start_time = time.time()
        
        try:
            while True:
                frame = camera.read_frame()
                
                if frame is None:
                    logger.warning("读取帧失败")
                    continue
                
                frame_count += 1
                
                # 添加信息到图像
                elapsed_time = time.time() - start_time
                if elapsed_time > 0:
                    fps = frame_count / elapsed_time
                    cv2.putText(frame, f"Frame: {frame_count}", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 70), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(frame, "New Multi-Process Camera", (10, 110), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                
                # 显示图像
                cv2.imshow('Multi-Process Camera Test', frame)
                
                # 处理按键
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    filename = f"multiprocess_camera_test_{int(time.time())}.jpg"
                    cv2.imwrite(filename, frame)
                    logger.info(f"图片已保存: {filename}")
        
        except KeyboardInterrupt:
            logger.info("用户中断程序")
        
        finally:
            cv2.destroyAllWindows()

if __name__ == "__main__":
    print("选择测试模式:")
    print("1. 性能测试 (无显示)")
    print("2. 可视化测试 (显示窗口)")
    
    try:
        choice = input("请输入选择 (1 或 2): ").strip()
        if choice == "1":
            test_camera_performance()
        elif choice == "2":
            test_visual_display()
        else:
            logger.error("无效选择")
    except KeyboardInterrupt:
        logger.info("程序被中断")
