#!/usr/bin/env python3

import rosbag
import numpy as np
import laspy
from tqdm import tqdm
import argparse
import os
import sensor_msgs.point_cloud2 as pc2

def convert_bag_to_las(bag_file, topic_name, output_file, max_points=None, downsample=1):
    """
    将ROS bag中的点云数据累积并转换为LAS文件
    
    参数:
        bag_file (str): 输入的bag文件路径
        topic_name (str): 点云topic名称
        output_file (str): 输出的LAS文件路径
        max_points (int, optional): 处理的最大点云数量，None表示全部处理
        downsample (int, optional): 降采样率，每隔几个点取一个
    """
    print(f"打开bag文件: {bag_file}")
    bag = rosbag.Bag(bag_file)
    
    # 获取消息总数以显示进度条
    msg_count = bag.get_message_count(topic_name)
    print(f"发现 {msg_count} 条消息在topic '{topic_name}'中")
    
    if max_points is not None:
        print(f"将最多处理 {max_points} 条消息")
        msg_count = min(msg_count, max_points)
    
    # 准备存储所有点云的列表
    all_points = []
    point_count = 0
    
    # 遍历bag中的点云消息
    print("提取点云数据...")
    for i, (_, msg, _) in enumerate(tqdm(bag.read_messages(topics=[topic_name]), total=msg_count)):
        if max_points is not None and i >= max_points:
            break
            
        # 将点云消息转换为numpy数组
        try:
            # 尝试处理PointCloud2消息
            if hasattr(msg, 'data') and hasattr(msg, 'fields'):
                # PointCloud2
                points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"))))
            elif hasattr(msg, 'points'):
                # PointCloud (旧版格式)
                points = np.array([[p.x, p.y, p.z] for p in msg.points])
            else:
                print(f"警告: 消息 {i} 不是支持的点云格式，跳过")
                continue
                
            # 应用降采样
            if downsample > 1:
                points = points[::downsample]
                
            # 将这批点添加到累积列表中
            all_points.append(points)
            point_count += len(points)
                
        except Exception as e:
            print(f"处理消息 {i} 时出错: {e}")
    
    bag.close()
    
    # 合并所有点云数据
    print(f"合并 {len(all_points)} 个点云，总共 {point_count} 个点...")
    if not all_points:
        print("错误: 没有找到有效的点云数据")
        return False
        
    all_points_array = np.vstack(all_points)
    print(f"累计的点云大小: {all_points_array.shape}")
    
    # 创建LAS文件
    print(f"创建LAS文件: {output_file}")
    las = laspy.create(file_version="1.4", point_format=7)
    
    # 设置坐标
    las.x = all_points_array[:, 0]
    las.y = all_points_array[:, 1]
    las.z = all_points_array[:, 2]
    
    # 保存LAS文件
    las.write(output_file)
    
    print(f"成功将点云数据保存到 {output_file}")
    print(f"LAS文件包含 {len(las.points)} 个点")
    
    return True

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='将ROS bag中的点云累积并转换为LAS文件')
    parser.add_argument('bag_file', help='输入的ROS bag文件路径')
    parser.add_argument('topic_name', help='点云topic名称')
    parser.add_argument('--output', '-o', help='输出的LAS文件路径 (默认: 与bag同名但后缀为.las)')
    parser.add_argument('--max-points', '-m', type=int, help='处理的最大点云消息数量')
    parser.add_argument('--downsample', '-d', type=int, default=1, help='降采样率 (默认: 1，不降采样)')
    
    args = parser.parse_args()
    
    # 如果未指定输出文件，则使用与输入文件相同的名称但后缀为.las
    if args.output is None:
        base_name = os.path.splitext(args.bag_file)[0]
        args.output = f"{base_name}.las"
    
    convert_bag_to_las(args.bag_file, args.topic_name, args.output, args.max_points, args.downsample)
