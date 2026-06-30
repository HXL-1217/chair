import cv2
from pyzbar import pyzbar
import time

def decode_qrcode_headless():
    # 初始化摄像头
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("❌ 无法打开摄像头，请检查设备！")
        return

    print("✅ 摄像头已启动 (纯终端模式)。")
    print("📷 请将二维码对准镜头... (按 Ctrl+C 停止)")

    # 设置画面分辨率（可选，降低分辨率可以减少算力消耗）
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    try:
        while True:
            # 读取每一帧画面
            ret, frame = cap.read()
            if not ret:
                print("⚠️ 无法获取画面流")
                time.sleep(1)
                continue

            # 使用 pyzbar 解析画面中的二维码
            barcodes = pyzbar.decode(frame)
            
            for barcode in barcodes:
                # 解码二维码中的数据
                barcodeData = barcode.data.decode("utf-8")
                barcodeType = barcode.type
                
                # 直接在 SSH 终端打印结果
                print(f"📦 [成功] 发现 {barcodeType}! 内容: {barcodeData}")
                
                # 稍微延时，防止同一个二维码在一秒内疯狂刷屏打印几百次
                time.sleep(0.5)

    except KeyboardInterrupt:
        # 捕捉你在终端按下 Ctrl+C 的动作
        print("\n🛑 已接收到退出指令。")
        
    finally:
        # 释放摄像头资源
        cap.release()
        print("🔌 摄像头已释放，程序安全退出。")

if __name__ == '__main__':
    decode_qrcode_headless()



# import cv2
# from pyzbar import pyzbar

# def decode_qrcode():
#     # 初始化摄像头。'0' 通常是默认的 USB 摄像头。
#     # 如果你用的是 MIPI 摄像头，或者有多个摄像头，这里可能需要改成 1, 2, 11 等
#     cap = cv2.VideoCapture(0)

#     if not cap.isOpened():
#         print("❌ 无法打开摄像头，请检查摄像头是否插好，或更改 VideoCapture 的设备号！")
#         return

#     print("✅ 摄像头已启动，请将二维码对准镜头。按 'q' 键退出...")

#     # 设置画面分辨率（可选）
#     cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#     cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

#     while True:
#         # 读取每一帧画面
#         ret, frame = cap.read()
#         if not ret:
#             print("无法获取画面流")
#             break

#         # 使用 pyzbar 解析画面中的二维码
#         barcodes = pyzbar.decode(frame)
        
#         for barcode in barcodes:
#             # 提取二维码的边界框坐标
#             (x, y, w, h) = barcode.rect
#             # 在画面上用绿色的线画出框框
#             cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

#             # 解码二维码中的数据 (默认是 bytes，需要转成 utf-8 字符串)
#             barcodeData = barcode.data.decode("utf-8")
#             barcodeType = barcode.type

#             # 在画面上显示解码出的文字
#             text = f"{barcodeData} ({barcodeType})"
#             cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
#             # 在终端打印结果
#             print(f"📦 发现二维码! 内容: {barcodeData}")

#         # 显示实时画面窗口
#         cv2.imshow("Orange Pi QR Code Scanner", frame)

#         # 监听键盘，如果按下 'q' 键则退出循环
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

#     # 释放摄像头并关闭所有窗口
#     cap.release()
#     cv2.destroyAllWindows()

# if __name__ == '__main__':
#     decode_qrcode()