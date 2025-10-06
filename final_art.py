import seekfree, pyb
import sensor, image, time, tf, gc
from machine import UART
uart = UART(2, baudrate=115200)     # 初始化串口 波特率设置为115200
sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)   # Set frame size to QVGA (320x240)
# sensor.set_auto_gain(False)         # 关闭自动增益
# sensor.set_auto_exposure(False, exposure_us=500)  # 设置曝光时间（单位：微秒）#500
# sensor.set_auto_whitebal(False)     # 关闭自动白平衡
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.

# 设置模型路径
detect1 = 'yolo3_iou_smartcar_final_with_post_processing.tflite' # - 7-15#-15-afternoon
#detect2 = '/sd/yolo3_nano_final_with_post_processing.tflite'
# 载入模型
net1 = tf.load(detect1)
# net2 = tf.load(detect2)

net_path2 = "16_class_ep7.tflite"  # 定义模型的路径 16分类   
labels2 = [line.rstrip() for line in open("/sd/labels2.txt")]   # 加载标签
net2 = tf.load(net_path2, load_to_fb=True)                                  # 加载模型

# 第三个模型（特殊分类模型）
net_path3 = "40w_ep3_valid.tflite" # 40w_ep3_valid.tflite 40w_ep4.tflite #last_ep6
labels3 = [line.rstrip() for line in open("/sd/40w_ep7.txt")]

net3 = tf.load(net_path3, load_to_fb=True)
# 全局状态变量
current_model = net2  # 当前使用的模型
current_labels = labels2  # 当前使用的标签
model_name = "Model2"  # 当前模型名称
special_mode = False  # 是否处于特殊模式（使用第三个模型）

# 分类结果统计
class ResultCounter:
    def __init__(self, buffer_size=3):
        self.buffer = []  # 存储最近分类结果的缓冲区
        self.buffer_size = buffer_size  # 缓冲区大小

    def add_result(self, class_index):
        """添加一个新的分类结果到缓冲区"""
        self.buffer.append(class_index)

        # 检查缓冲区是否已满
        if len(self.buffer) >= self.buffer_size:
            return self.process_results()
        return None

    def process_results(self):
        """处理缓冲区中的结果并返回统计值"""
        if not self.buffer:
            return None

        # 统计每个类别的出现次数
        count_dict = {}
        for idx in self.buffer:
            count_dict[idx] = count_dict.get(idx, 0) + 1

        # 找出出现次数最多的类别
        max_count = 0
        most_common_index = None

        for idx, count in count_dict.items():
            if count > max_count:
                max_count = count
                most_common_index = idx

        # 如果所有结果都不同，选择第一个结果
        if max_count == 1:
            most_common_index = self.buffer[0]

        # 清空缓冲区
        self.buffer = []

        return most_common_index

    def reset(self):
        """重置计数器"""
        self.buffer = []

# 发送结果函数
def send_result(class_index):
    """通过串口发送分类结果"""
    # 发送一个字节（16进制）
    uart.write(class_index.to_bytes(1, 'little'))  # 发送一个变量
    time.sleep(0.02)  # 确保数据发送完成
    uart.writechar(200) # 发送结束标志
    print(f"Sent class index: 0x{class_index:02X} ({class_index})")

# 检查是否需要进入特殊模式
def check_special_mode(result):
    """检查是否需要切换到第三个模型"""
    global current_model, current_labels, model_name, special_mode

    # 如果统计结果是6，进入特殊模式
    if result == 6:
        print("Switching to special model (Model3)")
        current_model = net3
        current_labels = labels3
        model_name = "Model3"
        special_mode = True
        special_counter.reset()  # 重置特殊模式计数器
    else:
        # 发送正常结果
        send_result(result)

# 创建结果计数器实例
result_counter = ResultCounter()  # 用于主分类结果统计
special_counter = ResultCounter()  # 用于特殊模式下的统计

while(True):
    clock.tick()
    img = sensor.snapshot()
    img1 = img.copy(0.75, 1)  # 创建缩放图像用于第一个模型

    # 存储第一个模型的检测结果
    first_stage_boxes = []

    # 使用第一个模型进行检测
    for obj in tf.detect(net1, img1):
        x1, y1, x2, y2, label, scores = obj
        if scores > 0.50:
            # 将归一化坐标转换为原始图像的实际坐标
            w = x2 - x1
            h = y2 - y1
            x1 = int(x1 * img.width())
            y1 = int(y1 * img.height())
            w = int(w * img.width())
            h = int(h * img.height())

            # 确保坐标不超出图像边界
            x1 = max(0, x1)
            y1 = max(0, y1)
            w = min(w, img.width() - x1)
            h = min(h, img.height() - y1)

            # 存储检测框信息
            first_stage_boxes.append((x1, y1, w, h))

            # 绘制第一个模型的检测框（蓝色）
            img.draw_rectangle((x1, y1, w, h), color=(255, 0, 0), thickness=2)

    # 在第一个模型的检测框内进行二次检测
    for (x, y, w, h) in first_stage_boxes:
        if w > 0 and h > 0:  # 确保ROI有效
            # 裁剪检测框区域
            roi = (x, y, w, h)
            subimg = img.copy(roi=roi)
            
            subimg.gamma_corr(brightness=-0.2)  # gamma 值越小，图像越暗
            #subimg.gamma(gamma=1.8, contrast=1.0, brightness=0.0)
                    # 方案2：直接乘法（备选）
            #subimg = subimg.multiply(0.6)  # 调整系数
            
            for obj in tf.classify(current_model , subimg, min_scale=1.0, scale_mul=0.5, x_overlap=0.0, y_overlap=0.0):
                print("**********\nTop 1 Detections at [x=%d,y=%d,w=%d,h=%d]" % obj.rect())
                #sorted_list = sorted(zip(labels, obj.output()), key = lambda x: x[1], reverse = True)
                scores = obj.output()  # 获取所有类别的分数列表
                top_index = scores.index(max(scores))  # 关键步骤：找到最高分对应的下标
                top_score = scores[top_index]
                top_label = current_labels[top_index]

                # 打印结果到控制台
                print("%s = %f,index = %d" % (top_label, top_score,top_index))

                # 在原始图像上显示分类结果 (在ROI框上方)
                display_text = "%s (%.1f%%)" % (top_label, top_score * 100)

                # 计算文本位置 (居中显示在ROI上方)
                text_width = len(display_text) * 10  # 估算文本宽度
                text_x = max(0, x + (w - text_width) // 2)
                text_y = max(10, y - 15)  # 在框上方显示

                # 绘制文本背景和文字
                # img.draw_rectangle((text_x-2, text_y-2, text_width+4, 20),
                #                  color=(0, 0, 0), fill=True)
                img.draw_string(text_x, text_y, display_text,
                              color=(0, 255, 0), scale=2, mono_space=False)
                # 根据当前模式处理结果
                if special_mode:
                    # 特殊模式：使用第三个模型，结果添加到特殊计数器
                    result = special_counter.add_result(top_index)
                    if result is not None:
                        # 特殊模式完成，发送结果并切换回正常模式
                        # result += 15
                        send_result(int(labels3[result])+16)
                        print("Returning to normal model (Model2)")
                        current_model = net2
                        current_labels = labels2
                        model_name = "Model2"
                        special_mode = False
                        result_counter.reset()  # 重置主计数器
                else:
                    # 正常模式：使用第二个模型，结果添加到主计数器
                    result = result_counter.add_result(top_index)
                    if result is not None:
                        # 处理主计数器结果
                        check_special_mode(result)
            # 使用第二个模型检测子图像
            # for obj2 in tf.detect(net2, subimg):
            #     x1_sub, y1_sub, x2_sub, y2_sub, label2, score2 = obj2
            #     if score2 > 0.30:  # 设置第二个模型的置信度阈值
            #         # 将子图像坐标转换回原始图像坐标
            #         sub_w = x2_sub - x1_sub
            #         sub_h = y2_sub - y1_sub
            #         abs_x = x + int(x1_sub * w)
            #         abs_y = y + int(y1_sub * h)
            #         abs_w = int(sub_w * w)
            #         abs_h = int(sub_h * h)

            #         # 绘制第二个模型的检测结果（绿色）
            #         img.draw_rectangle((abs_x, abs_y, abs_w, abs_h), color=(0, 255, 0), thickness=2)

            # 释放子图像内存
            del subimg
            gc.collect()

    # 显示帧率和当前模式
    mode_text = f"FPS:{clock.fps():.1f} Mode:{model_name}"
    img.draw_string(5, 5, mode_text, color=(0, 255, 0), scale=2)
    img.gamma_corr(brightness=-0.2)  # gamma 值越小，图像越暗
