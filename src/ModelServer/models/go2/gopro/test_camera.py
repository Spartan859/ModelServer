#!/usr/bin/env python3
"""
ITE LCC5046B 相机使用示例
简单的相机读取和显示示例
"""

from camera import USBCamera, detect_cameras
import cv2
import time


def simple_camera_test():
    """简单的相机测试函数"""
    print("=== ITE LCC5046B 相机测试 ===")
    
    # 检测相机
    cameras = detect_cameras()
    if not cameras:
        print("错误: 未检测到相机设备")
        return
    
    print(f"检测到相机: {cameras}")
    
    # 使用第一个相机
    camera = USBCamera(device_id=cameras[0], width=640, height=480, fps=30)
    
    if not camera.open():
        print("错误: 无法打开相机")
        return
    
    print("相机已成功打开!")
    print("相机信息:", camera.get_camera_info())
    
    # 读取10帧图像作为测试
    print("\n开始读取图像...")
    for i in range(10):
        success, frame = camera.read_frame()
        if success:
            print(f"成功读取第 {i+1} 帧, 图像大小: {frame.shape}")
            
            # 保存第一帧作为示例
            if i == 0:
                cv2.imwrite("test_frame.jpg", frame)
                print("已保存测试图像: test_frame.jpg")
        else:
            print(f"读取第 {i+1} 帧失败")
        
        time.sleep(0.1)  # 等待100ms
    
    camera.close()
    print("\n测试完成!")


def live_preview():
    """实时预览功能"""
    print("=== 实时相机预览 ===")
    print("按 'q' 退出, 按 's' 保存当前帧")
    
    cameras = detect_cameras()
    if not cameras:
        print("错误: 未检测到相机设备")
        return
    
    with USBCamera(device_id=cameras[0], width=640, height=480, fps=30) as camera:
        if not camera.is_opened:
            print("错误: 无法打开相机")
            return
        
        while True:
            success, frame = camera.read_frame()
            if not success:
                continue
            
            # 显示图像
            cv2.imshow('ITE Camera Live Preview', frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                filename = f"capture_{int(time.time())}.jpg"
                cv2.imwrite(filename, frame)
                print(f"已保存图像: {filename}")
    
    cv2.destroyAllWindows()


if __name__ == "__main__":
    # 运行简单测试
    simple_camera_test()
    
    # 询问是否要运行实时预览
    response = input("\n是否要运行实时预览? (y/n): ")
    if response.lower() == 'y':
        live_preview()
